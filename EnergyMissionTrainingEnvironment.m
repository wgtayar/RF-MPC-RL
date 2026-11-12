classdef EnergyMissionTrainingEnvironment < rl.env.MATLABEnvironment
    %EnergyMissionTrainingEnvironment Normalized action adapter with native capture.
    % The nested environment owns all robot, battery, reward and terminal
    % semantics. This adapter only applies an affine action transform and
    % archives the agent-side request before passing it to the native step.
    properties (SetAccess = private)
        Native
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
        end

        function observation = reset(obj,carriedEnergy)
            if nargin < 2, carriedEnergy = []; end
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
            [observation,reward,isDone,info] = step(obj.Native,physical);
            info.normalized_action = audit;
        end

        function close(obj,status)
            if nargin < 2, status='training_complete'; end
            if ~obj.Native.Writer.Closed
                close(obj.Native,status);
            end
        end
    end
end
