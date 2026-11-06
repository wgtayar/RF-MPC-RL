classdef testAttitudeTelemetry < matlab.unittest.TestCase
% Per-step body attitude telemetry (logged; not an observation input).
 methods(Test)
  function horizonReportsConsistentAttitude(tc)
   b=setup_energy_mission_rl(); c=b.cfg; s=initialize_mpc_replay_state(c,0);
   control=resolve_replay_action([zeros(3,1);c.GAMMA_V_MISSION;c.GAMMA_A_MIN],s,c,b.lower_abs,b.upper_abs);
   control.reference_mode='position_continuous_v1';
   o=struct('duration_s',.2,'update_battery',true,'update_proxy',false);
   [~,out]=simulate_mpc_horizon(s,control,c,o);
   a=out.attitude;
   tc.verifyEqual(a.integrated_steps,out.integrated_steps);
   tc.verifyEqual(a.tilt_threshold_rad,c.STABILITY.tilt_threshold_rad);
   rms=sqrt(a.tilt_square_sum/a.integrated_steps);
   tc.verifyGreaterThanOrEqual(a.max_tilt_rad,rms);
   tc.verifyGreaterThanOrEqual(a.tilt_excess_square_sum,0);
   tc.verifyLessThanOrEqual(a.tilt_exceedance_steps,a.integrated_steps);
   tc.verifyLessThan(a.max_tilt_rad,.1);
  end
  function windowAggregatesEveryChunkStep(tc)
   b=setup_energy_mission_rl(); c=b.cfg; s=initialize_mpc_replay_state(c,0);
   control=resolve_replay_action([zeros(3,1);c.GAMMA_V_MISSION;c.GAMMA_A_MIN],s,c,b.lower_abs,b.upper_abs);
   control.reference_mode='position_continuous_v1';
   o=struct('duration_s',.1,'update_battery',true,'update_proxy',false,'capture_trace',true);
   [s1,o1]=simulate_mpc_horizon(s,control,c,o); [s2,o2]=simulate_mpc_horizon(s1,control,c,o);
   s.decision_bookkeeping=struct('initial_position_x',s.Xt(1),'distance_m',0,'a_exec',control.a_cmd,'prev_Ieq_window',0);
   w=phase3_decision_window(s,s2,{o1,o2},control,c,"horizon_complete");
   steps=o1.attitude.integrated_steps+o2.attitude.integrated_steps;
   tc.verifyTrue(w.attitude.available);
   tc.verifyEqual(w.attitude.integrated_steps,steps);
   tc.verifyEqual(w.attitude.max_tilt_rad,max(o1.attitude.max_tilt_rad,o2.attitude.max_tilt_rad));
   tc.verifyEqual(w.attitude.rms_tilt_rad, ...
       sqrt((o1.attitude.tilt_square_sum+o2.attitude.tilt_square_sum)/steps),'AbsTol',1e-15);
  end
 end
end
