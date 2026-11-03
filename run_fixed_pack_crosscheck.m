function r=run_fixed_pack_crosscheck(root)
% Independent consumer, cell-replay and low-SOC checks. Write only fresh evidence.
assert(~isfolder(root) && ~isfile(root),'Output exists.'); mkdir(root);
b=setup_RL_MPC(); c=b.cfg; c.CHUNK_DURATION=.2; c.RUN.sim_snapshot_file=fullfile(root,'sim.mat');
s=initialize_mpc_replay_state(c,0); o=struct('duration_s',.2,'update_battery',true,'capture_trace',true);
ctl=struct('R',s.R,'v_cmd',.5,'a_cmd',.5,'reference_mode','position_continuous_v1');
[s,q]=simulate_mpc_horizon(s,ctl,c,o); reset_mpc_case_state();
h=run_MPC_simulation(repmat(s.R,4,1),0,.5,.5,c);
assert(h.feasible && q.completed_horizon && abs(h.battery.energy_Wh-s.energy.energyWh)<1e-10);
tr=h.battery.bms_input; [soc,~,~,~,~]=update_SOC_from_power_RL(tr);
assert(abs(soc-s.battery.soc_pct/100)<1e-10);
assert(max(abs(tr.PackVoltage.*tr.PackCurrent-tr.PowerW))<1e-10);
% Drive the pre-existing, independent batch 3RC/EKF with the streamed current.
time=[0;tr.Time]; current=[0;tr.CellCurrent];
[~,t,v]=model_battery(current,time,c.BATTERY.C_nom_Ah,c.BATTERY.SOC_init,false,c.BATTERY.model_options);
assert(isequal(t,tr.Time)); voltageError=max(abs(v-tr.CellVoltage));
socTrace=estimateSOC(time,current,[ppval(s.energy.ocv,c.BATTERY.SOC_init);v], ...
 c.BATTERY.C_nom_Ah,c.BATTERY.SOC_init,false,c.BATTERY.model_options);
socError=max(abs(socTrace(2:end)'-tr.SOC));
assert(voltageError<1e-10 && socError<1e-5);
c.BATTERY.SOC_init=.25; c.BATTERY.metric_init=25;
s=initialize_mpc_replay_state(c,0); o.duration_s=5; [s,z]=simulate_mpc_horizon(s,ctl,c,o);
r=struct('scope','consumer agreement, batch vs streaming cell comparison and 25 percent initial SOC; fixed 4S6P', ...
 'port_charge_error_As',abs(h.charge_total-q.charge_As),'voltage_error_V',voltageError, ...
 'estimator_error_percentage_points',socError,'low_soc_terminal',z.terminal_reason,'low_soc_seconds',s.t, ...
 'low_soc_qp_failures',z.qp_failed_count,'low_soc_pack_V',s.battery.pack_voltage_V,'low_soc_required_V',s.battery.drive_required_V);
[~,sha]=system('git rev-parse HEAD'); r.source_sha=strtrim(sha);
[~,dirty]=system('git status --porcelain'); r.dirty=~isempty(strtrim(dirty));
f=fopen(fullfile(root,'crosscheck.json'),'w'); clean=onCleanup(@()fclose(f)); fprintf(f,'%s\n',jsonencode(r,'PrettyPrint',true));
end
