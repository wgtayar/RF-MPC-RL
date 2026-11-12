classdef testRewardV3 < matlab.unittest.TestCase
    % Span-normalized reward v3: constants, telescoping to episode tau/kappa,
    % success in [1,2] for tau,kappa in [0,1], failure <= 0, backward terminal.
    methods(TestMethodSetup)
        function setup(~)
            bootstrap_RF_MPC_RL();
        end
    end
    methods(Test)
        function declaredConstants(tc)
            c = setup_energy_mission_rl().cfg; s = energy_mission_scales(c);
            tc.verifyEqual(c.REWARD.version,'reward_energy_mission_v3');
            tc.verifyEqual(s.reward_version,'reward_energy_mission_v3');
            tc.verifyEqual(c.ENERGY_RL.time_weight,.63);
            tc.verifyEqual(s.minimum_time_s,320/1.2,'AbsTol',1e-12);
            tc.verifyEqual(s.time_span_s,600-320/1.2,'AbsTol',1e-12);
            tc.verifyEqual(s.charge_floor_As_per_m*320,631.34,'AbsTol',.01);
            tc.verifyEqual(s.charge_span_As,84.53,'AbsTol',.01);
            tc.verifyEqual([s.completion_bonus,s.failure_penalty],[1 1]);
        end

        function windowsTelescopeToEpisodeTauKappa(tc)
            for w = [.3 .63 .8]
                c = setup_energy_mission_rl(struct('time_weight',w)).cfg; s = energy_mission_scales(c);
                T = 401; Q = 650; % arbitrary completed mission
                [G,last] = localEpisode(c,T,Q,320,'mission_complete',8);
                tau = (T-s.minimum_time_s)/s.time_span_s;
                kappa = (Q-s.charge_floor_As_per_m*320)/s.charge_span_As;
                tc.verifyEqual(G,2-w*tau-(1-w)*kappa,'AbsTol',1e-12);
                tc.verifyEqual(last.standardized_monitor.tau_cumulative,tau,'AbsTol',1e-12);
                tc.verifyEqual(last.standardized_monitor.kappa_cumulative,kappa,'AbsTol',1e-12);
            end
        end

        function successRangeAndFailureBound(tc)
            c = setup_energy_mission_rl().cfg; s = energy_mission_scales(c);
            best = localEpisode(c,s.minimum_time_s,s.charge_floor_As_per_m*320,320,'mission_complete',4);
            worst = localEpisode(c,600,s.charge_floor_As_per_m*320+s.charge_span_As,320,'mission_complete',4);
            tc.verifyEqual(best,2,'AbsTol',1e-12);
            tc.verifyEqual(worst,1,'AbsTol',1e-12);
            % A failure at any progress with nonnegative excess scores <= 0.
            for d = [0 100 319]
                failed = localEpisode(c,max(d/1.2,1),s.charge_floor_As_per_m*d,d,'actuator_limit',3);
                tc.verifyLessThanOrEqual(failed,1e-12);
            end
            backward = localEpisode(c,50,40,30,'backward_motion',2);
            tc.verifyLessThan(backward,0);
        end

        function netBackwardDecisionIsAFailureTerminal(tc)
            b = setup_energy_mission_rl(); c = b.cfg;
            c.PHASE3.options.mission_end_mode = 'mpc_step_target_v1';
            state = initialize_mpc_replay_state(c,0);
            origin = state.Xt(1);
            out = struct('terminal_reason',"horizon_complete",'completed_horizon',true, ...
                'qp_failed_count',0,'integrated_steps',500);
            % Backward after progress (50 m -> 49.5 m).
            state.decision_bookkeeping = struct('initial_position_x',origin,'distance_m',50);
            state.Xt(1) = origin+49.5;
            tc.verifyEqual(resolve_phase3_terminal(state,out,49.5,c),'backward_motion');
            state.Xt(1) = origin+52;
            tc.verifyEqual(resolve_phase3_terminal(state,out,52,c),'');
            % Backward behind the origin: the clipped distance is 0, the signed one -0.5 m.
            state.decision_bookkeeping = struct('initial_position_x',origin,'distance_m',0);
            state.Xt(1) = origin-.5;
            tc.verifyEqual(resolve_phase3_terminal(state,out,max(0,state.Xt(1)-origin),c),'backward_motion');
            state.Xt(1) = origin+1;
            tc.verifyEqual(resolve_phase3_terminal(state,out,1,c),'');
            % v2 configurations keep their historical terminal semantics.
            legacy = c; legacy.REWARD.version = 'reward_energy_mission_v2';
            state.Xt(1) = origin-.5;
            tc.verifyEqual(resolve_phase3_terminal(state,out,0,legacy),'');
        end
    end
end

function [G,info] = localEpisode(c,T,Q,D,terminal,n)
% Split an episode into n equal windows and sum v3 rewards.
G = 0;
for k = 1:n
    w = struct('duration_s',T/n,'charge_As',Q/n, ...
        'distance_start_m',D*(k-1)/n,'distance_end_m',D*k/n, ...
        'elapsed_time_s',T*k/n,'cumulative_charge_As',Q*k/n,'terminal_reason','');
    if k == n, w.terminal_reason = terminal; end
    [r,info] = compute_energy_mission_reward(w,c);
    G = G+r;
end
end
