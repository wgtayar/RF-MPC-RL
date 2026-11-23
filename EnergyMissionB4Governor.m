classdef EnergyMissionB4Governor < handle
    %EnergyMissionB4Governor One mission of the B4 headroom-feedback baseline.
    % A governor is created at each mission start. At every 5 s decision
    % act(native) reads, from the native environment (Phase3MpcEnvironment)
    % state, only the two signals the RL agent observes:
    %   estimated SOC                 State.energy.estimate(1)
    %                                 (observation entry estimated_soc)
    %   drive headroom per cell (V)   State.battery.drive_voltage_headroom_V
    %                                 divided by energy_mission_scales(cfg)
    %                                 .pack_voltage_unit_V (= n_series)
    %                                 (observation entry drive_headroom_per_series_cell)
    % The environment builds the agent's observation from the same state
    % after each decision (build_energy_mission_observation), so both are the
    % raw, unnormalized values behind the agent's inputs. From the second
    % decision on, act() asserts that they equal the raw entries recorded in
    % the last transition's observation audit. The speed command follows
    % energy_mission_b4_headroom_speed; the action keeps dR = 0 and
    % gamma_a = GAMMA_A_MIN, mapped by the evaluation runner's speedAction.
    properties (SetAccess = private)
        Params
        Command = NaN
        InitialCommand = NaN
        Log = struct([])
    end
    properties (Access = private)
        SpeedAction
        Unit
        SocIndex
        HeadroomIndex
    end
    methods
        function obj = EnergyMissionB4Governor(params,speedAction,initialCommand)
            % initialCommand (optional, m/s): soft-start command held until the
            % first headroom reading (sensitivity variant b4_v_start_m_s). NaN
            % or omitted: the frozen B4, whose first decision commands min(v*, cap).
            energy_mission_b4_headroom_speed(NaN,NaN,1,params);   % validates params
            obj.Params = params;
            obj.SpeedAction = speedAction;
            if nargin >= 3 && ~isnan(initialCommand)
                validateattributes(initialCommand,{'double'},{'scalar','real','finite', ...
                    '>=',params.v_min_m_s,'<=',params.v_max_m_s});
                obj.Command = initialCommand;
                obj.InitialCommand = initialCommand;
            end
        end

        function action = act(obj,native)
            if isempty(obj.Unit)
                % The environment's own configuration defines the observation.
                cfg = native.Config;
                assert(strcmp(cfg.PHASE3.observation_schema,'observation_energy_mission_v1'), ...
                    'b4Governor:Schema','B4 reads the energy-mission observation signals.');
                obj.Unit = energy_mission_scales(cfg).pack_voltage_unit_V;
                names = energy_mission_observation_schema(cfg).names;
                obj.SocIndex = find(names=="estimated_soc");
                obj.HeadroomIndex = find(names=="drive_headroom_per_series_cell");
                assert(isscalar(obj.SocIndex) && isscalar(obj.HeadroomIndex),'b4Governor:Schema', ...
                    'Observation schema lacks estimated_soc or drive_headroom_per_series_cell.');
            end
            s = native.State.energy.estimate(1);
            headroom = NaN;
            b = native.State.battery;
            if isfield(b,'drive_voltage_headroom_V') && isscalar(b.drive_voltage_headroom_V) && ...
                    isfinite(b.drive_voltage_headroom_V)
                headroom = b.drive_voltage_headroom_V/obj.Unit;
            end
            if native.Decision > 0
                raw = native.LastTransition.observation_audit.raw_unmasked;
                assert(isequaln(raw(obj.SocIndex),s) && isequaln(raw(obj.HeadroomIndex),headroom), ...
                    'b4Governor:Signal','B4 signals differ from the agent observation audit.');
            end
            previous = obj.Command;
            [obj.Command,audit] = energy_mission_b4_headroom_speed(previous,headroom,s,obj.Params);
            audit.decision = native.Decision+1;
            audit.time_s = native.State.t;
            audit.previous_executed_speed_m_s = native.State.supervisory_state.v_exec;
            if isempty(obj.Log), obj.Log = audit; else, obj.Log(end+1) = audit; end
            action = obj.SpeedAction(obj.Command);
        end

        function out = summary(obj)
            % Per-mission counts. mean_command_m_s is the unweighted mean of
            % the B4 speed commands over the mission's decisions.
            log = obj.Log;
            h = [log.headroom_per_cell_V];
            out = struct('decisions',numel(log),'mean_command_m_s',mean([log.command_m_s]), ...
                'min_command_m_s',min([log.command_m_s]),'final_command_m_s',log(end).command_m_s, ...
                'speed_reductions',sum([log.reduced]),'speed_increases',sum([log.increased]), ...
                'cap_limited_decisions',sum([log.cap_limited]), ...
                'rate_limited_decisions',sum([log.rate_limited]), ...
                'min_headroom_per_cell_V',min([h(isfinite(h)) NaN]));
        end
    end
end
