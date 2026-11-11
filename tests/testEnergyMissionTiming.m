classdef testEnergyMissionTiming < matlab.unittest.TestCase
    % Cadence and governor qualification without advancing the robot plant.
    % Reachability here concerns commanded speed, not physical feasibility.
    properties
        Bundle
    end
    methods (TestMethodSetup)
        function configure(tc)
            previous = rng;
            cleanup = onCleanup(@() rng(previous)); %#ok<NASGU>
            bootstrap_RF_MPC_RL();
            tc.Bundle = setup_energy_mission_rl();
        end
    end
    methods (Test)
        function defaultCadencePreservesMissionAndLockedPlant(tc)
            c = tc.Bundle.cfg;
            tc.verifyEqual(c.ENERGY_RL.options.decision_interval_s,5);
            tc.verifyEqual([c.CHUNK_DURATION,c.APPLY_EVERY,c.EP_STEPS],[5,1,120]);
            tc.verifyEqual([c.MISSION.D_TARGET_M,c.MISSION_DURATION],[320,600]);
            tc.verifyEqual(c.MISSION.WINDOW_TARGET_M,320/120,'AbsTol',1e-14);
            tc.verifyEqual(c.BATTERY.fixed_pack_spec.id, ...
                'beyond_torque_9s1p_18650_nancy_2ah_20261106_v1');
            tc.verifyEqual([c.BATTERY.n_series,c.BATTERY.n_parallel,c.BATTERY.C_nom_Ah],[9,1,2]);
            tc.verifyEqual([c.ENERGY.auxW,c.ENERGY.inverterEfficiency],[15,.90]);
            tc.verifyEqual(c.DGAMMA_V_MAX*(c.V_MAX-c.V_MIN),.024,'AbsTol',1e-14);
            tc.verifyEqual(c.DR_MAX,.0005);
        end

        function testedSpeedRequestsAreReachableWithinDefaultHorizon(tc)
            b = tc.Bundle; c = b.cfg;
            targets = [.8,1,1.2];
            expectedActions = [12,20,28];
            expectedApplicationTimes = [55,95,135];
            for j = 1:numel(targets)
                state = initialize_mpc_replay_state(c,0);
                state.R = b.initial_R;
                initialSpeed = state.supervisory_state.v_exec;
                tc.verifyEqual(initialSpeed,320/600,'AbsTol',1e-14);
                gamma = (targets(j)-c.V_MIN)/(c.V_MAX-c.V_MIN);
                candidate = [zeros(3,1);gamma;c.GAMMA_A_MIN];
                commanded = zeros(expectedActions(j)+1,1);
                for k = 1:numel(commanded)
                    [control,execution] = resolve_replay_action(candidate,state,c,b.lower_abs,b.upper_abs);
                    commanded(k) = control.v_cmd;
                    tc.verifyEqual(control.R,b.initial_R,'AbsTol',1e-14);
                    state = acceptSupervisor(state,control,execution);
                end
                first = find(abs(commanded-targets(j))<1e-12,1);
                tc.verifyEqual(first,expectedActions(j));
                tc.verifyEqual((first-1)*c.CHUNK_DURATION*c.APPLY_EVERY,expectedApplicationTimes(j));
                tc.verifyLessThan(first,c.EP_STEPS);
                changes = diff([initialSpeed;commanded]);
                tc.verifyGreaterThanOrEqual(min(changes),-1e-14);
                tc.verifyLessThanOrEqual(max(changes),.024+1e-14);
                tc.verifyEqual(commanded(end),targets(j),'AbsTol',1e-12);
            end
        end

        function reversingExtremeRequestsRetainsRateAndAbsoluteBounds(tc)
            b = tc.Bundle; c = b.cfg;
            state = initialize_mpc_replay_state(c,0);
            state.R = b.initial_R;
            % Deliberately out-of-range requests exercise the production
            % rate limiter and saturation in both directions.
            for requestedGamma = [2,-1]
                before = state.supervisory_state.v_exec;
                commanded = zeros(45,1);
                candidate = [zeros(3,1);requestedGamma;c.GAMMA_A_MIN];
                for k = 1:numel(commanded)
                    [control,execution] = resolve_replay_action(candidate,state,c,b.lower_abs,b.upper_abs);
                    commanded(k) = control.v_cmd;
                    tc.verifyEqual(execution.previous_applied_action,state.previous_action,'AbsTol',1e-14);
                    state = acceptSupervisor(state,control,execution);
                end
                changes = diff([before;commanded]);
                tc.verifyLessThanOrEqual(max(abs(changes)),.024+1e-14);
                tc.verifyGreaterThanOrEqual(min(commanded),c.V_MIN-1e-14);
                tc.verifyLessThanOrEqual(max(commanded),c.V_MAX+1e-14);
                if requestedGamma>1
                    tc.verifyGreaterThanOrEqual(min(changes),-1e-14);
                    tc.verifyEqual(commanded(end),c.V_MAX,'AbsTol',1e-14);
                else
                    tc.verifyLessThanOrEqual(max(changes),1e-14);
                    tc.verifyEqual(commanded(end),c.V_MIN,'AbsTol',1e-14);
                end
            end
        end

        function explicitHistoricalCadenceAndLegacyProfileRemainAvailable(tc)
            previous = rng;
            cleanup = onCleanup(@() rng(previous)); %#ok<NASGU>
            b = setup_energy_mission_rl(struct('decision_interval_s',50));
            c = b.cfg; current = tc.Bundle.cfg;
            tc.verifyEqual([c.CHUNK_DURATION,c.APPLY_EVERY,c.EP_STEPS],[5,10,12]);
            tc.verifyEqual(c.ENERGY_RL.options.decision_interval_s,50);
            tc.verifyEqual(c.DGAMMA_V_MAX,current.DGAMMA_V_MAX);
            tc.verifyEqual(c.DR_MAX,current.DR_MAX);
            tc.verifyEqual(c.BATTERY,current.BATTERY);
            tc.verifyEqual(c.ENERGY,current.ENERGY);
            legacy = setup_RL_MPC('legacy_frozen_v1'); c = legacy.cfg;
            tc.verifyEqual([c.CHUNK_DURATION,c.APPLY_EVERY,c.EP_STEPS],[5,10,12]);
            tc.verifyEqual([c.V_MIN,c.V_MAX,c.GAMMA_V_MAX],[.3,1.1,.45]);
            tc.verifyEqual(c.DGAMMA_V_MAX,.03);
            tc.verifyEqual(c.DR_MAX,.0005);
        end
    end
end

function state = acceptSupervisor(state,control,execution)
% Update only the action history as simulate_mpc_horizon does after a chunk.
% No elapsed time, dynamics, energy or battery state is fabricated here.
state.R = control.R;
state.previous_action = control.action;
state.supervisory_state = struct('last_R',execution.R_applied, ...
    'v_req',execution.v_req,'a_req',execution.a_req, ...
    'v_exec',execution.v_exec,'prev_gamma_v',execution.gamma_v_applied, ...
    'prev_gamma_a',execution.gamma_a_applied, ...
    'previous_applied_action',execution.applied_action);
end
