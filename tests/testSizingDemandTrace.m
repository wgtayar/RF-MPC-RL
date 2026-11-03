classdef testSizingDemandTrace < matlab.unittest.TestCase
 methods(Test)
  function diagnosticsDoNotChangeTransition(tc)
   b=setup_RL_MPC(); c=b.cfg; s=initialize_mpc_replay_state(c,0);
   control=struct('R',s.R,'v_cmd',.5,'a_cmd',.5,'reference_mode','position_continuous_v1');
   options=struct('duration_s',.03,'update_proxy',false,'update_battery',true);
   [a,plain]=simulate_mpc_horizon(s,control,c,options);
   options.capture_drive_trace=true;
   [z,logged]=simulate_mpc_horizon(s,control,c,options);
   tc.verifyEqual(a,z);
   tc.verifyEqual(logged.charge_As,plain.charge_As);
   tc.verifySize(logged.drive_trace,[3,53]);
   tc.verifySize(logged.drive_inputs,[3,114]);
   tc.verifyEqual(logged.drive_inputs(end,31:60)',a.Xt);
   d=logged.drive_trace;
   tc.verifyEqual(sum(d(:,3).*(d(:,2)-d(:,1)))/3600,logged.energy_Wh,'AbsTol',1e-14);
   tc.verifyEqual(max(abs(d(:,42:53)),[],2),d(:,4),'AbsTol',1e-14);
   tc.verifyEqual(d(:,42:53),c.ENERGY.lineResistance*d(:,30:41)+ ...
       2*c.ENERGY.fluxPolePairs*d(:,18:29).*repmat(c.ENERGY.ratio',1,4),'AbsTol',1e-13);
  end
 end
end
