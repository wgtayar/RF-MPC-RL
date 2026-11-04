function result=run_fixed_pack_mission_review(target,deadline,policy)
% Fixed pack, production governor/reference/solver, stop at the first violation.
b=setup_RL_MPC('legacy_frozen_v1'); c=b.cfg; s=initialize_mpc_replay_state(c,0);
origin=s.Xt(1); attempts=0; failed=0; rescued=0; peakTilt=0; decisions=0;
reason="time_limit";
while s.t < deadline-1e-8
    if mod(round(s.t*100),round(100*c.CHUNK_DURATION*c.APPLY_EVERY))==0
        switch policy
            case 'mission_average', g=c.GAMMA_V_MISSION;
            case 'fast', g=c.GAMMA_V_MAX;
            case 'slow', g=(.5-c.V_MIN)/(c.V_MAX-c.V_MIN);
            otherwise, error('Unknown fixed policy.');
        end
        ctl=resolve_replay_action([zeros(3,1);g;c.GAMMA_A_MIN],s,c,b.lower_abs,b.upper_abs);
        ctl.reference_mode='position_continuous_v1'; decisions=decisions+1;
    end
    opt=struct('duration_s',min(c.CHUNK_DURATION,deadline-s.t),'stop_at_position_x_m',origin+target, ...
        'capture_trace',true,'update_battery',true,'update_proxy',false, ...
        'solver_strategy','default_one_shot_fallback_tight_primal_v1');
    [s,o]=simulate_mpc_horizon(s,ctl,c,opt);
    attempts=attempts+o.qp_solve_count; failed=failed+o.qp_failed_count;
    rescued=rescued+nnz(o.trace.solver_classification=="numerical_solver_failure_recovered");
    peakTilt=max(peakTilt,max(o.trace.orientation_error_after_rad));
    if o.terminal_reason~="horizon_complete", reason=o.terminal_reason; break; end
    if s.battery.margin_norm<=c.BATTERY.terminal_margin, reason="battery_terminal"; break; end
end
result=struct('policy',policy,'target_m',target,'deadline_s',deadline,'terminal',reason, ...
    'time_s',s.t,'distance_m',s.Xt(1)-origin,'decisions',decisions,'solves',attempts, ...
    'failed',failed,'rescued',rescued,'peak_orientation_rad',peakTilt,'battery',s.battery);
end
