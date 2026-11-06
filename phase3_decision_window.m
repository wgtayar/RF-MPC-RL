function window = phase3_decision_window(before, after, chunks, control, cfg, terminal)
%phase3_decision_window Aggregate captured chunks without discarding failed work.
    bookkeeping = before.decision_bookkeeping;
    % Invalid raw states remain in the dataset. Only terminal observation/reward
    % features use this explicit last-valid fallback, never the physical state.
    invalidXt = ~isreal(after.Xt) || any(~isfinite(after.Xt(:)));
    invalidUt = ~isreal(after.Ut) || any(~isfinite(after.Ut(:)));
    batteryValues = [after.battery.margin_norm,after.battery.soc_pct];
    invalidBattery = ~isreal(batteryValues) || any(~isfinite(batteryValues));
    invalidFeatures = invalidXt || invalidUt || invalidBattery;
    if invalidXt
        after.Xt = before.Xt;
    end
    if invalidBattery
        after.battery = before.battery;
    end
    distance = max(0,after.Xt(1)-bookkeeping.initial_position_x);
    duration = after.t-before.t;
    charge = sum(cellfun(@(out) out.charge_As,chunks));
    if isfield(cfg,'ENERGY_RL')
        assert(abs(charge-(after.energy.chargeAs-before.energy.chargeAs))<1e-8, ...
            'energyMission:ChargeAccounting','Chunks must book the same physical charge used by SOC.');
    end
    integrated = cellfun(@(out) out.integrated_steps,chunks) > 0;
    tracking = cellfun(@(out) out.tracking_error_sum,chunks);
    effort = cellfun(@(out) out.control_effort_sum,chunks);
    window = struct();
    window.invalid_state_feature_fallback = invalidFeatures;
    window.tracking_error_mean = 1e3;
    window.control_effort_mean = 1e6;
    if any(integrated)
        % Preserve the legacy mean-of-chunk-sums units for the bridge audit.
        window.tracking_error_mean = mean(tracking(integrated));
        window.control_effort_mean = mean(effort(integrated));
    end
    window.Ieq_window = cfg.IEQ_REF;
    if isfield(cfg,'ENERGY_RL'),window.Ieq_window=0;end
    if duration > 0
        window.Ieq_window = charge/duration;
    end
    window.charge_As = charge;
    if ~isempty(chunks) && all(cellfun(@(out) isfield(out,'attitude'),chunks))
        window.attitude = localAttitude(chunks);
    end
    if isfield(after.battery,'energy_Wh')
        window.energy_Wh=after.battery.energy_Wh-before.battery.energy_Wh;
    end
    window.duration_s = duration;
    window.elapsed_time_s=after.t;
    if isfield(after,'energy'),window.cumulative_charge_As=after.energy.chargeAs;end
    window.battery = after.battery;
    window.v_req = control.action_execution.v_req;
    window.a_req = control.action_execution.a_req;
    window.v_exec = control.v_cmd;
    window.a_exec = control.a_cmd;
    window.distance_start_m = bookkeeping.distance_m;
    window.window_distance_m = after.Xt(1)-before.Xt(1);
    window.distance_end_m = distance;
    window.progress_frac = min(distance/cfg.MISSION.D_TARGET_M,1);
    window.time_frac = min(after.t/cfg.MISSION_DURATION,1);
    window.lag_frac = max(0,window.time_frac-window.progress_frac);
    window.soc_start_pct = before.battery.soc_pct;
    window.soc_end_pct = after.battery.soc_pct;
    window.delta_v_exec = control.action_execution.delta_v_exec;
    window.delta_gamma_v = control.action_execution.delta_gamma_v_applied;
    window.delta_a_exec = control.a_cmd-bookkeeping.a_exec;
    window.delta_gamma_a = control.action_execution.gamma_a_applied- ...
        before.supervisory_state.prev_gamma_a;
    window.prev_Ieq_window = bookkeeping.prev_Ieq_window;
    window.dR1 = control.action_execution.dR_clipped(1);
    window.dR2 = control.action_execution.dR_clipped(2);
    window.dR3 = control.action_execution.dR_clipped(3);
    window.com_speed_mag = norm(after.Xt(4:5));
    window.state_norm_proxy = norm(after.Xt);
    window.tst_ratio = chunks{end}.stance_duration_end_s/cfg.OBS.NOMINAL_TST;
    window.fsm_proxy = chunks{end}.fsm_end(1)-1;
    if ~isreal([window.tst_ratio,window.fsm_proxy]) || ...
            ~isfinite(window.tst_ratio) || ~isfinite(window.fsm_proxy)
        window.invalid_state_feature_fallback = true;
        window.tst_ratio = 1;
        window.fsm_proxy = 0;
    end
    window.terminal_reason = terminal;
    window.completed_episode = strcmp(terminal,'mission_complete');
    window.feasible = all(cellfun(@(out) out.completed_horizon || ...
        (strcmp(out.terminal_reason,'mission_target_reached') && ...
        out.qp_failed_count==0 && out.integrated_steps>0),chunks));
    window.recovered_solver_events = sum(cellfun(@(out) ...
        nnz(out.trace.solver_classification == "numerical_solver_failure_recovered"),chunks));
    window.qp_solve_count = sum(cellfun(@(out) out.qp_solve_count,chunks));
end

function attitude = localAttitude(chunks)
% Whole-window body attitude from every integrated MPC step of every chunk.
    attitude = struct('available',false,'integrated_steps',0,'max_tilt_rad',NaN, ...
        'rms_tilt_rad',NaN,'max_body_rate_rad_s',NaN,'tilt_exceedance_fraction',NaN, ...
        'tilt_threshold_rad',NaN,'rms_tilt_excess_rad',NaN);
    has = cellfun(@(out) isfield(out,'attitude'),chunks);
    if isempty(chunks) || ~all(has), return, end
    a = cellfun(@(out) out.attitude,chunks,'UniformOutput',false); a = [a{:}];
    steps = sum([a.integrated_steps]);
    thresholds = unique([a.tilt_threshold_rad]);
    assert(numel(thresholds)==1,'phase3window:AttitudeThreshold', ...
        'Chunks of one decision must share the tilt threshold.');
    attitude.tilt_threshold_rad = thresholds;
    attitude.integrated_steps = steps;
    if steps==0, return, end
    attitude.available = true;
    attitude.max_tilt_rad = max([a.max_tilt_rad]);
    attitude.rms_tilt_rad = sqrt(sum([a.tilt_square_sum])/steps);
    attitude.max_body_rate_rad_s = max([a.max_body_rate_rad_s]);
    attitude.tilt_exceedance_fraction = sum([a.tilt_exceedance_steps])/steps;
    attitude.rms_tilt_excess_rad = sqrt(sum([a.tilt_excess_square_sum])/steps);
end
