classdef EnergyMissionTrainingEnvironment < rl.env.MATLABEnvironment
    %EnergyMissionTrainingEnvironment Normalized action adapter with native capture.
    % The nested environment owns all robot, battery, reward and terminal
    % semantics. This adapter only applies an affine action transform and
    % archives the agent-side request before passing it to the native step.
    properties (SetAccess = private)
        Native
        Shift = struct('enabled',false,'max_missions',4,'min_start_soc',.30)
        ShiftEpisode = 0          % learner (toolbox) episode counter
        MissionsInEpisode = 0     % missions completed in the current learner episode
        ShiftId = 0               % one shift = one battery from a fresh start SOC to its end
        PendingObservation = []   % first observation of a mission already started at a cap
        MissionStartSoc = NaN
    end
    methods
        function obj = EnergyMissionTrainingEnvironment(bundle,root,metadata,options)
            if nargin < 4, options=struct(); end
            assert(isfield(bundle.cfg,'ENERGY_RL'), ...
                'EnergyMissionTrainingEnvironment:Version','An explicit energy mission config is required.');
            expected = struct('observation_schema','observation_energy_mission_v1', ...
                'reward_schema',bundle.cfg.REWARD.version, ...
                'reference_mode',configured_reference_mode(bundle.cfg), ...
                'solver_strategy','default_one_shot_fallback_tight_primal_v1', ...
                'mission_end_mode','mpc_step_target_v1');
            for name=fieldnames(expected).'
                if isfield(options,name{1})
                    assert(strcmp(options.(name{1}),expected.(name{1})), ...
                        'EnergyMissionTrainingEnvironment:Schema', ...
                        'The captured energy-mission training schema and solver modes are fixed.');
                end
                options.(name{1})=expected.(name{1});
            end
            metadata.agent_action_schema = 'energy_mission_normalized_action_v1';
            native = Phase3MpcEnvironment(bundle,root,metadata,options);
            obsInfo = getObservationInfo(native);
            actInfo = rlNumericSpec([5,1],'LowerLimit',-ones(5,1), ...
                'UpperLimit',ones(5,1),'Name','normalized_supervisory_candidate');
            obj = obj@rl.env.MATLABEnvironment(obsInfo,actInfo);
            obj.Native = native;
            mkdir(fullfile(native.Writer.Root,'normalized_actions'));
            o = bundle.cfg.ENERGY_RL.options;
            if isfield(o,'shift') && o.shift.enabled
                obj.Shift = o.shift;
                fid = fopen(fullfile(native.Writer.Root,'shift_missions.csv'),'w');
                assert(fid>=0,'EnergyMissionTrainingEnvironment:ShiftLog','Cannot create the shift log.');
                fprintf(fid,'shift,learner_episode,mission_in_episode,native_episode,start_soc_true,end_soc_true,end_soc_estimated,end_reason,learner_done\n');
                fclose(fid);
            end
        end

        function observation = reset(obj,carriedEnergy)
            if nargin < 2, carriedEnergy = []; end
            obj.ShiftEpisode = obj.ShiftEpisode+1;
            obj.MissionsInEpisode = 0;
            if obj.Shift.enabled && ~isempty(obj.PendingObservation)
                % Previous learner episode was truncated at the mission cap: the
                % next mission (carried battery) has already started; continue it.
                observation = obj.PendingObservation;
                obj.PendingObservation = [];
                return
            end
            if obj.Shift.enabled, obj.ShiftId = obj.ShiftId+1; end
            observation = reset(obj.Native,carriedEnergy);
        end

        function [observation,reward,isDone,info] = step(obj,action)
            [physical,audit] = energy_mission_action_transform(action,obj.Native.Config);
            audit.episode = obj.Native.Episode;
            audit.decision = obj.Native.Decision+1;
            audit.rng_state = rng;
            audit.native_record = sprintf('decisions/episode_%06d_decision_%06d.mat', ...
                audit.episode,audit.decision);
            path = fullfile(obj.Native.Writer.Root,'normalized_actions', ...
                sprintf('episode_%06d_decision_%06d.mat',audit.episode,audit.decision));
            assert(~isfile(path),'EnergyMissionTrainingEnvironment:DuplicateAction', ...
                'An action audit already exists for this episode and decision.');
            save(path,'audit');
            startSoc = obj.Native.State.energy.soc;
            if obj.Native.Decision == 0, obj.MissionStartSoc = startSoc; end
            [observation,reward,isDone,info] = step(obj.Native,physical);
            info.normalized_action = audit;
            if obj.Shift.enabled && isDone
                [observation,isDone,info] = obj.chainMission(observation,info);
            end
        end

        function [observation,isDone,info] = chainMission(obj,observation,info)
            % Learner-side episode semantics for shift training (monitor #40).
            % The native mission has ended. Decide whether the learner's episode
            % continues (next mission, battery carried), is truncated at the cap
            % (IsDone=2: the critic bootstraps), or ends (IsDone=1).
            e = obj.Native.State.energy;
            nativeEpisode = obj.Native.Episode;
            reason = char(info.terminal_reason);
            if strcmp(reason,'mission_complete')
                obj.MissionsInEpisode = obj.MissionsInEpisode+1;
                if e.estimate(1) < obj.Shift.min_start_soc
                    isDone = 1;                      % shift over: no further mission possible
                else
                    carried = struct('soc',e.soc,'rc',e.rc,'estimate',e.estimate,'cov',e.cov);
                    observation = reset(obj.Native,carried);   % robot restarts, battery carried
                    if obj.MissionsInEpisode >= obj.Shift.max_missions
                        isDone = 2;                  % cap: truncation, continue next episode
                        obj.PendingObservation = observation;
                    else
                        isDone = 0;                  % same learner episode, next mission
                    end
                end
            else
                isDone = 1;                          % mission failure ends the shift
            end
            info.shift = struct('shift',obj.ShiftId,'learner_episode',obj.ShiftEpisode, ...
                'mission_in_episode',obj.MissionsInEpisode,'native_episode',nativeEpisode, ...
                'learner_done',isDone);
            fid = fopen(fullfile(obj.Native.Writer.Root,'shift_missions.csv'),'a');
            if fid>=0
                fprintf(fid,'%d,%d,%d,%d,%.9f,%.9f,%.9f,%s,%d\n',obj.ShiftId,obj.ShiftEpisode, ...
                    obj.MissionsInEpisode,nativeEpisode,obj.MissionStartSoc,e.soc,e.estimate(1),reason,isDone);
                fclose(fid);
            end
            obj.MissionStartSoc = obj.Native.State.energy.soc;
        end

        function close(obj,status)
            if nargin < 2, status='training_complete'; end
            if ~obj.Native.Writer.Closed
                close(obj.Native,status);
            end
        end
    end
end
