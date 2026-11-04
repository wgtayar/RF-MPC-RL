function report=run_energy_review(speeds,duration)
% Deterministic open-loop qualification, no training or sealed data.
if nargin<1, speeds=[.5 .66 .70]; end
if nargin<2, duration=5; end
bundle=setup_RL_MPC('legacy_frozen_v1'); cfg=bundle.cfg;
rows=cell(numel(speeds),1);
for j=1:numel(speeds)
 s=initialize_mpc_replay_state(cfg,0);
 c=struct('R',s.R,'v_cmd',speeds(j),'a_cmd',.5,'reference_mode','position_continuous_v1');
 opts=struct('duration_s',duration,'enforce_drive_limits',false,'capture_trace',true,'update_proxy',false,'update_battery',true, ...
   'solver_strategy','default_one_shot_fallback_tight_primal_v1');
 [s,o]=simulate_mpc_horizon(s,c,cfg,opts);
 rows{j}=[speeds(j),s.t,s.Xt(1),o.qp_solve_count,o.qp_failed_count, ...
 nnz(o.trace.solver_classification=="numerical_solver_failure_recovered"), ...
 max(o.trace.orientation_error_after_rad),s.energy.energyWh,s.energy.chargeAs, ...
 s.battery.soc_pct,s.energy.voltageLimitSteps,s.energy.torqueLimitSteps];
end
report=array2table(vertcat(rows{:}),'VariableNames',{'command','time','x','solves','failures','rescues','peakTilt','Wh','As','socPct','voltageFlags','torqueFlags'});
end
