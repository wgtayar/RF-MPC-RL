classdef testPhase3InvalidStateContract < matlab.unittest.TestCase
    % Synthetic public-interface contracts; no simulator, QP, or dataset writes.
    properties
        Config
    end
    properties (TestParameter)
        invalidCase = struct('complex_position','complex_position', ...
            'complex_velocity','complex_velocity','complex_control','complex_control', ...
            'nan_control','nan_control','inf_state','inf_state')
        batteryField = struct('margin','margin_norm','soc','soc_pct')
    end
    methods (TestClassSetup)
        function configure(testCase)
            source = bootstrap_RF_MPC_RL();
            bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg');
            cfg = bundle.cfg;
            cfg.RANDOMIZE_REQUEST = false;
            cfg.BATTERY.use_pack_sizing = false;
            cfg.CHUNK_DURATION = 0.03;
            cfg.APPLY_EVERY = 1;
            cfg.MISSION_DURATION = 1;
            cfg.MISSION.D_TARGET_M = 1;
            cfg.PHASE3.options = struct();
            testCase.Config = cfg;
        end
    end
    methods (Test)
        function invalidHealthPreservesRawInputs(testCase,invalidCase)
            f = localFixture(testCase.Config,invalidCase);
            raw = f.after;
            desired = f.desired;

            health = exact_mpc_health(f.after.Xt,f.after.Ut,f.desired);

            testCase.verifyFalse(health.valid);
            testCase.verifyEmpty(fieldnames(health.components));
            testCase.verifySize(health.per_leg_force_norms,[1 4]);
            testCase.verifyTrue(all(isnan(health.per_leg_force_norms)));
            testCase.verifyTrue(isnan(health.linear_velocity_error));
            testCase.verifyTrue(isnan(health.Ut_norm));
            testCase.verifyTrue(isequaln(f.after,raw));
            testCase.verifyTrue(isequaln(f.desired,desired));
        end
        function invalidTerminalPrecedesMissionBatteryAndTime(testCase,invalidCase)
            f = localFixture(testCase.Config,invalidCase);
            state = f.after;
            state.t = testCase.Config.MISSION_DURATION;
            state.battery.margin_norm = testCase.Config.BATTERY.terminal_margin;
            out = struct('completed_horizon',true,'terminal_reason','horizon_complete');

            [reason,done] = resolve_phase3_terminal(state,out, ...
                testCase.Config.MISSION.D_TARGET_M,testCase.Config);

            testCase.verifyEqual(reason,'invalid_state');
            testCase.verifyTrue(done);
        end
        function directWindowFlagsInvalidStateAndKeepsRawState(testCase,invalidCase)
            f = localFixture(testCase.Config,invalidCase);
            rawBefore = f.before;
            rawAfter = f.after;

            window = phase3_decision_window(f.before,f.after,{f.out}, ...
                f.control,testCase.Config,'invalid_state');

            testCase.verifyTrue(window.invalid_state_feature_fallback);
            testCase.verifyTrue(localRealFiniteWindow(window));
            testCase.verifyEqual(window.terminal_reason,'invalid_state');
            testCase.verifyFalse(window.completed_episode);
            testCase.verifyFalse(window.feasible);
            testCase.verifyEqual(window.distance_end_m,f.expectedDistance,AbsTol=1e-12);
            testCase.verifyEqual(window.duration_s,0.03,AbsTol=1e-12);
            testCase.verifyEqual(window.charge_As,0.1,AbsTol=1e-12);
            testCase.verifyTrue(isequaln(f.before,rawBefore));
            testCase.verifyTrue(isequaln(f.after,rawAfter));
        end
        function reconstructedInvalidRowRetainsHealthyPrefix(testCase,invalidCase)
            f = localFixture(testCase.Config,invalidCase);
            rawRows = f.rows;
            rawAfter = f.after;

            [window,terminal,chunk,audit] = localReconstruct(f,testCase.Config);
            direct = phase3_decision_window(f.before,f.after,{f.out}, ...
                f.control,testCase.Config,'invalid_state');

            testCase.verifyEqual(terminal,'invalid_state');
            testCase.verifyTrue(audit.is_done);
            testCase.verifyFalse(audit.saved_window_or_outcome_used);
            testCase.verifyEqual(chunk.out.terminal_reason,"invalid_state");
            testCase.verifyFalse(chunk.out.completed_horizon);
            testCase.verifyEqual(chunk.out.integrated_steps,2);
            testCase.verifyEqual(chunk.out.qp_solve_count,3);
            testCase.verifyEqual(chunk.out.qp_failed_count,0);
            testCase.verifyEqual(chunk.out.tracking_error_sum,f.expectedTracking,AbsTol=1e-12);
            testCase.verifyEqual(chunk.out.control_effort_sum,24,AbsTol=1e-12);
            testCase.verifyTrue(f.rows{3}.integrated);
            testCase.verifyFalse(f.rows{3}.current_sample_committed);
            testCase.verifyEqual(f.after.current_time,[0;0.01],AbsTol=1e-12);
            testCase.verifyEqual(f.after.current_total,[10;10],AbsTol=1e-12);
            testCase.verifyEqual(window,direct,AbsTol=1e-12);
            testCase.verifyTrue(isequaln(f.rows,rawRows));
            testCase.verifyTrue(isequaln(f.after,rawAfter));
        end
        function observationMarksInvalidDynamicsAndRemainsFinite(testCase,invalidCase)
            f = localFixture(testCase.Config,invalidCase);
            rawAfter = f.after;
            window = phase3_decision_window(f.before,f.after,{f.out}, ...
                f.control,testCase.Config,'invalid_state');
            legacy = localLegacy(window,f.control,testCase.Config);

            [observation,audit] = build_phase3_observation_v2(f.after,legacy, ...
                f.rows{3},testCase.Config,'observation_v2_dynamic_health_candidate_v2');

            testCase.verifySize(observation,[76 1]);
            testCase.verifyTrue(isreal(observation) && all(isfinite(observation)));
            testCase.verifyEqual(observation(audit.schema.names=="dynamic_state_valid"),0);
            testCase.verifyFalse(any(audit.available(ismember(audit.schema.names, ...
                ["velocity_x","orientation_angle","omega_x","force_norm_1"]))));
            testCase.verifyTrue(isequaln(f.after,rawAfter));
        end
        function streamingRewardRetainsCostsAndRejectsPositivePace(testCase,invalidCase)
            cfg = testCase.Config;
            cfg.MISSION.D_TARGET_M = 0.01;
            f = localFixture(cfg,invalidCase);
            rawAfter = f.after;
            [window,terminal] = localReconstruct(f,cfg);
            exposure = localExposure(f);
            policy = phase3reward.unitPolicy('unit_reference_experimental_v1');
            originalPace = f.expectedDistance/cfg.MISSION.D_TARGET_M-0.03;

            [reward,info] = compute_phase3_reward_candidate(window, ...
                f.control.action_execution,exposure,f.before,f.after,cfg,policy);

            testCase.verifyEqual(terminal,'invalid_state');
            testCase.verifyTrue(isreal(reward) && isfinite(reward));
            testCase.verifyLessThan(reward,0);
            testCase.verifyLessThanOrEqual(info.pace_contribution,0);
            testCase.verifyEqual(info.pace_contribution,min(originalPace,0),AbsTol=1e-12);
            testCase.verifyEqual(info.positive_pace_removed(3),max(originalPace,0),AbsTol=1e-12);
            testCase.verifyEqual(info.actual_terminal_reason,"invalid_state");
            testCase.verifyEqual(info.terminal_contribution,-1,AbsTol=1e-12);
            testCase.verifyFalse(info.physical_costs_complete);
            testCase.verifyEqual(exposure.integrated_duration_s,0.03,AbsTol=1e-12);
            testCase.verifyEqual(exposure.finite_coverage_s,0.02*ones(1,3),AbsTol=1e-12);
            testCase.verifyEqual(exposure.missing_coverage_s,0.01*ones(1,3),AbsTol=1e-12);
            testCase.verifyEqual(exposure.known_squared_integrals,[0.0008,0.08,0.0002],AbsTol=1e-12);
            testCase.verifyTrue(all(isnan(exposure.full_squared_integrals)));
            testCase.verifyGreaterThan(info.weighted_known_costs(1:4),zeros(1,4));
            testCase.verifyEqual(info.weighted_missing_exposure_costs,0.01*ones(1,3),AbsTol=1e-12);
            testCase.verifyEqual(info.charge_audit.booked_charge_As,0.1,AbsTol=1e-12);
            testCase.verifyTrue(isequaln(f.after,rawAfter));
        end
        function healthyControlRemainsValidThroughout(testCase)
            f = localFixture(testCase.Config,'healthy');
            health = exact_mpc_health(f.after.Xt,f.after.Ut,f.desired);
            [window,terminal,chunk,audit] = localReconstruct(f,testCase.Config);
            exposure = localExposure(f);
            legacy = localLegacy(window,f.control,testCase.Config);

            [observation,observationAudit] = build_phase3_observation_v2(f.after,legacy, ...
                f.rows{3},testCase.Config,'observation_v2_dynamic_health_candidate_v2');
            [reward,info] = compute_phase3_reward_candidate(window, ...
                f.control.action_execution,exposure,f.before,f.after,testCase.Config, ...
                phase3reward.unitPolicy('unit_reference_experimental_v1'));

            testCase.verifyTrue(health.valid);
            testCase.verifyNotEmpty(fieldnames(health.components));
            testCase.verifyTrue(all(isfinite([health.per_leg_force_norms, ...
                health.linear_velocity_error,health.Ut_norm])));
            testCase.verifyEqual(terminal,'');
            testCase.verifyFalse(audit.is_done);
            testCase.verifyFalse(window.invalid_state_feature_fallback);
            testCase.verifyTrue(window.feasible && localRealFiniteWindow(window));
            testCase.verifyEqual(chunk.out.integrated_steps,3);
            testCase.verifyEqual(window.distance_end_m,0.003,AbsTol=1e-12);
            testCase.verifyEqual(window.charge_As,0.2,AbsTol=1e-12);
            testCase.verifyTrue(f.rows{3}.current_sample_committed);
            testCase.verifyTrue(exposure.physical_complete);
            testCase.verifyEqual(exposure.known_squared_integrals,[0.0012,0.12,0.0003],AbsTol=1e-12);
            testCase.verifyEqual(exposure.missing_coverage_s,zeros(1,3),AbsTol=1e-12);
            testCase.verifyTrue(isreal(observation) && all(isfinite(observation)));
            testCase.verifyEqual(observation(observationAudit.schema.names=="dynamic_state_valid"),1);
            testCase.verifyTrue(isreal(reward) && isfinite(reward));
            testCase.verifyTrue(info.physical_costs_complete);
            testCase.verifyEqual(info.terminal_contribution,0,AbsTol=1e-12);
        end
        function complexBatteryTerminatesAndUsesWindowFallback(testCase,batteryField)
            f = localFixture(testCase.Config,'healthy');
            f.after.battery.(batteryField) = f.after.battery.(batteryField)+1i;
            rawAfter = f.after;

            [terminal,done] = resolve_phase3_terminal(f.after,f.out, ...
                testCase.Config.MISSION.D_TARGET_M,testCase.Config);
            window = phase3_decision_window(f.before,f.after,{f.out}, ...
                f.control,testCase.Config,terminal);

            testCase.verifyEqual(terminal,'invalid_state');
            testCase.verifyTrue(done);
            testCase.verifyTrue(window.invalid_state_feature_fallback);
            testCase.verifyTrue(localRealFiniteWindow(window));
            testCase.verifyEqual(window.battery,f.before.battery,AbsTol=1e-12);
            testCase.verifyEqual(window.soc_end_pct,f.before.battery.soc_pct,AbsTol=1e-12);
            testCase.verifyEqual(window.distance_end_m,0.003,AbsTol=1e-12);
            testCase.verifyTrue(isequaln(f.after,rawAfter));
        end
    end
end

function f = localFixture(cfg,kind)
    gamma = [cfg.GAMMA_V_MISSION;cfg.GAMMA_A_MIN];
    [velocity,~] = apply_command_governor(cfg.V_REQ_FIXED,cfg.A_REQ_FIXED,gamma(1),gamma(2),cfg);
    previous = struct('last_R',ones(3,1),'v_req',cfg.V_REQ_FIXED,'a_req',cfg.A_REQ_FIXED, ...
        'v_exec',velocity,'prev_gamma_v',gamma(1),'prev_gamma_a',gamma(2), ...
        'previous_applied_action',[zeros(3,1);gamma]);
    execution = resolve_supervisory_action([zeros(3,1);gamma],previous,cfg, ...
        0.95*ones(3,1),1.05*ones(3,1));
    control = struct('action_execution',execution,'R',execution.R_applied, ...
        'v_cmd',execution.v_exec,'a_cmd',execution.a_exec);
    rotation = [cos(0.2),-sin(0.2),0;sin(0.2),cos(0.2),0;0,0,1];
    desired = [zeros(6,1);reshape(eye(3),9,1);zeros(15,1)];
    Xt = desired;
    Xt(7:15) = rotation(:);
    Xt(4) = 0.1;
    Xt(16) = 2;
    before = struct('Xt',Xt,'Ut',ones(12,1),'t',0,'gait',0, ...
        'current_time',zeros(0,1),'current_total',zeros(0,1), ...
        'battery',struct('soc_pct',80,'margin_norm',0.8),'supervisory_state',previous, ...
        'fsm_internal_state',struct('FSM',ones(4,1),'Ta',zeros(4,1),'Tb',0.2*ones(4,1)), ...
        'decision_bookkeeping',struct('initial_position_x',0,'distance_m',0, ...
        'prev_Ieq_window',0,'a_exec',execution.a_exec));
    context = struct('episode',1,'decision',1,'chunk',1);
    snapshot = struct('state',before,'parameters',struct('simTimeStep',0.01,'Tst',0.2), ...
        'control',control,'context',context);
    rows = cell(3,1);
    state = before;
    for k = 1:3
        row = struct('row_id',k,'segment',1,'context',context,'iteration',k, ...
            'time_before',state.t,'state_before',compact_exact_mpc_state(state), ...
            'Xt_qp',state.Xt,'Ut_qp',state.Ut,'Xd',desired,'FSM',ones(4,1), ...
            'action_execution',execution,'integrated',true, ...
            'solver',struct('success',true,'classification','solver_success', ...
            'iterations',1,'wall_time_s',0,'diagnostics',struct('inequality_margin_min',1)), ...
            'current_sample_committed',true,'current_sample_time',state.t,'current_sample_A',10);
        state.t = k*0.01;
        state.Xt(1) = k*0.001;
        if k==3 && ~strcmp(kind,'healthy')
            state = localInvalidate(state,kind);
            row.current_sample_committed = false;
        end
        if row.current_sample_committed
            state.current_time(end+1,1) = row.current_sample_time;
            state.current_total(end+1,1) = row.current_sample_A;
        end
        row.state_after = compact_exact_mpc_state(state);
        row.time_after = state.t;
        rows{k} = row;
    end
    healthy = strcmp(kind,'healthy');
    successful = 2+double(healthy);
    tracking = successful*(4*(1-cos(0.2))+0.1^2+2^2)+sum(((1:successful)*0.001).^2);
    reason = "invalid_state";
    if healthy
        reason = "horizon_complete";
    end
    out = struct('terminal_reason',reason,'completed_horizon',healthy, ...
        'integrated_steps',successful,'qp_solve_count',3,'qp_failed_count',0, ...
        'tracking_error_sum',tracking,'control_effort_sum',12*successful, ...
        'fsm_end',ones(4,1),'stance_duration_end_s',0.2, ...
        'charge_As',0.1*(successful-1),'trace',struct('solver_classification',strings(0,1)));
    expectedDistance = 0.003;
    if any(strcmp(kind,{'complex_position','complex_velocity','inf_state'}))
        expectedDistance = 0;
    end
    f = struct('before',before,'after',state,'desired',desired,'control',control, ...
        'snapshot',snapshot,'rows',{rows},'out',out,'expectedTracking',tracking, ...
        'expectedDistance',expectedDistance);
end

function state = localInvalidate(state,kind)
    switch kind
        case 'complex_position'
            state.Xt(1) = state.Xt(1)+1i;
        case 'complex_velocity'
            state.Xt(4) = state.Xt(4)+1i;
        case 'complex_control'
            state.Ut(1) = state.Ut(1)+1i;
        case 'nan_control'
            state.Ut(1) = NaN;
        case 'inf_state'
            state.Xt(16) = Inf;
        otherwise
            error('testPhase3InvalidStateContract:Case','Unknown synthetic invalid case.');
    end
end

function [window,terminal,chunk,audit] = localReconstruct(f,cfg)
    accumulator = [];
    for k = 1:numel(f.rows)
        accumulator = phase3reward.accumulateChunk(accumulator,f.rows{k},f.snapshot,cfg);
    end
    chunk = phase3reward.finishChunk(accumulator,f.after);
    [window,terminal,audit] = phase3reward.reconstructWindow({chunk},cfg);
end

function exposure = localExposure(f)
    accumulator = [];
    for k = 1:numel(f.rows)
        row = f.rows{k};
        row.health_before = exact_mpc_health(row.Xt_qp,row.Ut_qp,row.Xd);
        row.health_after = exact_mpc_health(row.state_after.Xt,row.state_after.Ut,row.Xd);
        accumulator = phase3reward.accumulate(accumulator,row,0.01);
    end
    exposure = phase3reward.finishExposure(accumulator);
end

function legacy = localLegacy(w,control,cfg)
    legacy = build_rl_observation(w.tracking_error_mean/cfg.TRACK_REF, ...
        w.control_effort_mean/cfg.EFFORT_REF,w.battery,w.progress_frac,w.lag_frac, ...
        w.v_req,w.a_req,w.v_exec,w.a_exec,control.R,0.95*ones(3,1),1.05*ones(3,1),cfg, ...
        w.com_speed_mag,w.tst_ratio,w.state_norm_proxy,w.fsm_proxy, ...
        control.action_execution.gamma_v_applied,control.action_execution.gamma_a_applied,w.Ieq_window);
end

function valid = localRealFiniteWindow(window)
    names = fieldnames(window);
    values = [];
    for k = 1:numel(names)
        value = window.(names{k});
        if isnumeric(value)
            values = [values;value(:)]; %#ok<AGROW>
        end
    end
    values = [values;window.battery.margin_norm;window.battery.soc_pct];
    valid = isreal(values) && all(isfinite(values));
end
