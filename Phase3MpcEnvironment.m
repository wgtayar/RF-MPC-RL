classdef Phase3MpcEnvironment < rl.env.MATLABEnvironment
    %Phase3MpcEnvironment Isolated, fully captured supervisory RL interface.
    % Constructor inputs are an explicit runtime bundle, new dataset path,
    % provenance metadata and optional reference_mode/solver_strategy options.
    % Uses an explicit observation schema and a comparison-only reward bridge. This is
    % not promoted for training until observation/reward/viability gates pass.
    properties (SetAccess = private)
        State = struct()
        Config
        Writer
        Options
        Episode = 0
        Decision = 0
        Done = true
        Observation = []
        LastTransition = struct()
    end
    properties (Access = private)
        InitialR
        LowerR
        UpperR
        ResetStream
    end
    methods
        function obj = Phase3MpcEnvironment(bundle, root, metadata, options)
            if nargin < 4
                options = struct();
            end
            assert(all(isfield(bundle,{'cfg','initial_R','lower_abs','upper_abs'})), ...
                'Phase3MpcEnvironment:Bundle','Explicit config, initial R and bounds are required.');
            cfg = bundle.cfg;
            options = localOptions(options);
            p = get_params(0);
            validateattributes(cfg.APPLY_EVERY,{'double'},{'scalar','integer','positive'});
            validateattributes(cfg.MISSION_DURATION,{'double'},{'scalar','finite','positive'});
            validateattributes(cfg.CHUNK_DURATION,{'double'},{'scalar','finite','positive'});
            assert(abs(cfg.CHUNK_DURATION/p.simTimeStep-round(cfg.CHUNK_DURATION/p.simTimeStep)) < 1e-8 && ...
                abs(cfg.MISSION_DURATION/p.simTimeStep-round(cfg.MISSION_DURATION/p.simTimeStep)) < 1e-8, ...
                'Phase3MpcEnvironment:DurationGrid','Chunk and mission durations must be on the MPC grid.');
            validateattributes(bundle.initial_R,{'double'},{'numel',3,'finite','positive'});
            validateattributes(bundle.lower_abs,{'double'},{'numel',3,'finite','positive'});
            validateattributes(bundle.upper_abs,{'double'},{'numel',3,'finite','positive'});
            assert(all(bundle.lower_abs(:) < bundle.upper_abs(:)) && ...
                all(bundle.initial_R(:) >= bundle.lower_abs(:)) && ...
                all(bundle.initial_R(:) <= bundle.upper_abs(:)), ...
                'Phase3MpcEnvironment:Bounds','Initial R must be inside ordered, nondegenerate bounds.');
            observationSchema = 'observation_v1_legacy';
            observationDimension = 19;
            if isfield(options,'observation_schema')
                observationSchema = options.observation_schema;
            end
            if ~strcmp(observationSchema,'observation_v1_legacy')
                v2 = phase3_observation_v2_schema(cfg,observationSchema);
                observationDimension = v2.dimension;
            end
            observationInfo = rlNumericSpec([observationDimension,1],'Name','observations');
            actionInfo = rlNumericSpec([5,1], ...
                'LowerLimit',[-cfg.DR_MAX*ones(3,1);cfg.GAMMA_V_MIN;cfg.GAMMA_A_MIN], ...
                'UpperLimit',[cfg.DR_MAX*ones(3,1);cfg.GAMMA_V_MAX;cfg.GAMMA_A_MAX], ...
                'Name','supervisory_candidate');
            obj = obj@rl.env.MATLABEnvironment(observationInfo,actionInfo);
            cfg.PHASE3 = struct('environment_version','phase3_captured_environment_v1', ...
                'observation_schema',observationSchema, ...
                'reward_version','reward_phase3_legacy_bridge_v1', ...
                'training_promoted',false,'reset_R_each_episode',true, ...
                'options',options,'initial_R',bundle.initial_R(:), ...
                'lower_R',bundle.lower_abs(:),'upper_R',bundle.upper_abs(:));
            metadata.observation_schema_version = cfg.PHASE3.observation_schema;
            metadata.reward_version = cfg.PHASE3.reward_version;
            metadata.solver_strategy = options.solver_strategy;
            metadata.environment_version = cfg.PHASE3.environment_version;
            metadata.training_promoted = false;
            metadata.supervisory_records_required = true;
            metadata.seed = cfg.RNG_SEED;
            obj.Config = cfg;
            obj.InitialR = bundle.initial_R(:);
            obj.LowerR = bundle.lower_abs(:);
            obj.UpperR = bundle.upper_abs(:);
            obj.Options = options;
            obj.ResetStream = RandStream('mt19937ar','Seed',cfg.RNG_SEED);
            obj.Writer = ExactStateDataset(root,cfg,metadata);
        end

        function observation = reset(obj)
            obj.requireOpen();
            if obj.Episode > 0 && ~obj.Done
                obj.recordEpisodeEnd('reset_interrupted');
            end
            cfg = obj.Config;
            obj.State = initialize_mpc_replay_state(cfg,0);
            initialConditionAudit = struct();
            if isfield(obj.Options,'initial_condition')
                [obj.State,initialConditionAudit] = apply_phase3_initial_condition( ...
                    obj.State,obj.Options.initial_condition);
            end
            obj.State.R = obj.InitialR;
            obj.State.supervisory_state.last_R = obj.InitialR;
            [vReq,aReq] = obj.sampleRequest();
            obj.State.supervisory_state.v_req = vReq;
            obj.State.supervisory_state.a_req = aReq;
            supervisor = obj.State.supervisory_state;
            observation = build_rl_observation(0,0,obj.State.battery,0,0, ...
                vReq,aReq,supervisor.v_exec,aReq,obj.InitialR,obj.LowerR,obj.UpperR,cfg, ...
                0,1,0,0,supervisor.prev_gamma_v,supervisor.prev_gamma_a,0);
            obj.Episode = obj.Episode+1;
            obj.Decision = 0;
            obj.Done = false;
            obj.Observation = observation;
            obj.LastTransition = struct();
            obj.State.decision_bookkeeping = struct('episode',obj.Episode,'decision',0, ...
                'initial_position_x',obj.State.Xt(1),'distance_m',0, ...
                'prev_Ieq_window',0,'a_exec',aReq,'observation',observation, ...
                'reset_stream_state',obj.ResetStream.State);
            observationAudit = struct();
            if obj.usesObservationV2()
                [observation,observationAudit] = build_phase3_observation_v2(obj.State,observation,struct(), ...
                    cfg,cfg.PHASE3.observation_schema);
                obj.Observation = observation;
                obj.State.decision_bookkeeping.observation = observation;
            end
            record = struct('state',obj.State,'observation',observation, ...
                'lifecycle_reason','reset','rng_state',rng,'observation_audit',observationAudit);
            if isfield(obj.Options,'initial_condition')
                record.initial_condition_audit = initialConditionAudit;
            end
            obj.Writer.writeEpisode(obj.Episode,'start',record);
        end

        function [observation,reward,isDone,info] = step(obj,action)
            obj.requireOpen();
            assert(~obj.Done,'Phase3MpcEnvironment:ResetRequired','Reset before stepping a new episode.');
            validateattributes(action,{'double','single'},{'numel',5,'finite','real'});
            before = obj.State;
            control = resolve_replay_action(double(action(:)),before,obj.Config,obj.LowerR,obj.UpperR);
            control.reference_mode = obj.Options.reference_mode;
            firstRow = obj.Writer.RowCount+1;
            obj.Decision = obj.Decision+1;
            try
                [chunks,terminal] = obj.advanceDecision(control);
                window = phase3_decision_window(before,obj.State,chunks,control,obj.Config,terminal);
                [reward,rewardInfo] = compute_phase3_reward_bridge(window,obj.Config);
                [observation,observationAudit] = obj.observeWindow(window,control);
                assert(isfinite(reward) && all(isfinite(observation)), ...
                    'Phase3MpcEnvironment:NonfiniteTransition','Reward/observation must be finite.');
                obj.Observation = observation;
                obj.State.decision_bookkeeping.decision = obj.Decision;
                obj.State.decision_bookkeeping.distance_m = window.distance_end_m;
                obj.State.decision_bookkeeping.prev_Ieq_window = window.Ieq_window;
                obj.State.decision_bookkeeping.a_exec = control.a_cmd;
                obj.State.decision_bookkeeping.observation = observation;
                isDone = ~isempty(terminal);
                obj.Done = isDone;
                info = struct('state',obj.State,'observation',before.decision_bookkeeping.observation, ...
                    'next_observation',observation,'action_execution',control.action_execution, ...
                    'reward',reward,'reward_info',rewardInfo,'terminal_reason',terminal, ...
                    'is_done',isDone,'window',window,'first_mpc_row',firstRow, ...
                    'last_mpc_row',obj.Writer.RowCount,'chunks_completed',numel(chunks), ...
                    'rng_state',rng,'safety_mapping','none_candidate_semantics', ...
                    'projection_enabled',false,'projection_distance',0,'observation_audit',observationAudit);
                obj.Writer.writeDecision(obj.Episode,obj.Decision,info);
                obj.LastTransition = info;
                if isDone
                    obj.recordEpisodeEnd(terminal);
                end
            catch exception
                obj.Done = true;
                record = struct('state',obj.State,'observation',obj.Observation, ...
                    'lifecycle_reason','environment_exception', ...
                    'state_may_precede_failure',true,'exception_identifier',exception.identifier, ...
                    'exception_message',exception.message);
                obj.Writer.writeEpisode(obj.Episode,'end',record);
                obj.Writer.finish('environment_exception');
                rethrow(exception)
            end
        end

        function observation = resumeFromDecision(obj,sourceDataset,episode,decision)
            obj.requireOpen();
            assert(obj.Episode == 0 && obj.Writer.RowCount == 0, ...
                'Phase3MpcEnvironment:ResumeRequiresNewRun','Resume only into a fresh capture run.');
            validateattributes([episode,decision],{'numeric'},{'numel',2,'integer','positive'});
            validate_exact_state_dataset(sourceDataset);
            savedConfig = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
            assert(isequaln(savedConfig.cfg,obj.Config), ...
                'Phase3MpcEnvironment:ResumeConfig','Exact resume requires the identical archived configuration.');
            relative = sprintf('decisions/episode_%06d_decision_%06d.mat',episode,decision);
            saved = load(fullfile(sourceDataset,relative),'event');
            record = saved.event.record;
            assert(~record.is_done,'Phase3MpcEnvironment:ResumeTerminal','Cannot resume a terminal decision.');
            obj.State = record.state;
            obj.Episode = episode;
            obj.Decision = decision;
            obj.Done = false;
            obj.Observation = record.next_observation;
            obj.LastTransition = record;
            obj.ResetStream.State = obj.State.decision_bookkeeping.reset_stream_state;
            observation = obj.Observation;
            obj.Writer.writeEpisode(episode,'start',struct('state',obj.State, ...
                'observation',observation,'lifecycle_reason','resume_from_decision', ...
                'source_dataset',sourceDataset,'source_record',relative, ...
                'source_record_sha256',sha256_file(fullfile(sourceDataset,relative))));
        end

        function close(obj,status)
            if nargin < 2
                status = 'evaluation_complete';
            end
            obj.requireOpen();
            if obj.Episode > 0 && ~obj.Done
                obj.recordEpisodeEnd('run_closed_before_terminal');
            end
            obj.Done = true;
            obj.Writer.finish(status);
        end
    end
    methods (Access = private)
        function [chunks,terminal] = advanceDecision(obj,control)
            cfg = obj.Config;
            chunks = cell(cfg.APPLY_EVERY,1);
            terminal = '';
            for k = 1:cfg.APPLY_EVERY
                options = struct('duration_s',min(cfg.CHUNK_DURATION,cfg.MISSION_DURATION-obj.State.t), ...
                    'solver_strategy',obj.Options.solver_strategy,'capture_trace',true, ...
                    'update_battery',true,'dataset_writer',obj.Writer, ...
                    'dataset_context',struct('episode',obj.Episode,'decision',obj.Decision,'chunk',k));
                [obj.State,chunks{k}] = simulate_mpc_horizon(obj.State,control,cfg,options);
                distance = max(0,obj.State.Xt(1)-obj.State.decision_bookkeeping.initial_position_x);
                [terminal,finished] = resolve_phase3_terminal(obj.State,chunks{k},distance,cfg);
                if finished
                    break
                end
            end
            chunks = chunks(1:k);
        end

        function [observation,audit] = observeWindow(obj,window,control)
            cfg = obj.Config;
            observation = build_rl_observation( ...
                window.tracking_error_mean/cfg.TRACK_REF,window.control_effort_mean/cfg.EFFORT_REF, ...
                window.battery,window.progress_frac,window.lag_frac,window.v_req,window.a_req, ...
                window.v_exec,window.a_exec,control.R,obj.LowerR,obj.UpperR,cfg, ...
                window.com_speed_mag,window.tst_ratio,window.state_norm_proxy,window.fsm_proxy, ...
                control.action_execution.gamma_v_applied,control.action_execution.gamma_a_applied, ...
                window.Ieq_window);
            audit = struct();
            if obj.usesObservationV2()
                [observation,audit] = build_phase3_observation_v2(obj.State,observation,obj.Writer.lastRow(), ...
                    cfg,cfg.PHASE3.observation_schema);
            end
        end

        function enabled = usesObservationV2(obj)
            enabled = ~strcmp(obj.Config.PHASE3.observation_schema,'observation_v1_legacy');
        end

        function [vReq,aReq] = sampleRequest(obj)
            cfg = obj.Config;
            vReq = cfg.V_REQ_FIXED;
            aReq = cfg.A_REQ_FIXED;
            if cfg.RANDOMIZE_REQUEST
                vReq = cfg.V_MIN+(cfg.V_MAX-cfg.V_MIN)*rand(obj.ResetStream);
                low = max(cfg.A_MIN,vReq/cfg.TACC_MAX);
                high = min(cfg.A_MAX,vReq/cfg.TACC_MIN);
                aReq = low+(high-low)*rand(obj.ResetStream);
            end
        end

        function recordEpisodeEnd(obj,reason)
            record = struct('state',obj.State,'observation',obj.Observation, ...
                'lifecycle_reason',reason,'rng_state',rng);
            obj.Writer.writeEpisode(obj.Episode,'end',record);
        end

        function requireOpen(obj)
            assert(~obj.Writer.Closed,'Phase3MpcEnvironment:Closed','The captured run is finalized.');
        end
    end
end

function options = localOptions(options)
    assert(all(ismember(fieldnames(options),{'reference_mode','solver_strategy','observation_schema','initial_condition'})), ...
        'Phase3MpcEnvironment:Options','Unknown environment option.');
    if isfield(options,'initial_condition')
        options.initial_condition = validate_phase3_initial_condition(options.initial_condition);
    end
    if ~isfield(options,'reference_mode')
        options.reference_mode = 'legacy_absolute_time';
    end
    if ~isfield(options,'solver_strategy')
        options.solver_strategy = 'default';
    end
    options.reference_mode = validatestring(options.reference_mode, ...
        {'legacy_absolute_time','position_continuous_v1'});
    options.solver_strategy = validatestring(options.solver_strategy, ...
        {'default','default_one_shot_fallback','active_set_feasible_point', ...
        'default_one_shot_fallback_tight_primal_v1','active_set_tight_primal_v1'});
    if isfield(options,'observation_schema')
        options.observation_schema = validatestring(options.observation_schema, ...
            {'observation_v1_legacy','observation_v2_dynamic_health_candidate_v1', ...
            'observation_v2_dynamic_health_candidate_v2'});
    end
end
