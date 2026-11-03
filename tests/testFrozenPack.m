classdef testFrozenPack < matlab.unittest.TestCase
 methods(Test)
  function sizeChangesAreRejected(tc)
   b=phase3battery.fixedPackConfig();
   for name={'n_series','n_parallel','C_nom_Ah'}
    changed=b; changed.(name{1})=changed.(name{1})+1;
    tc.verifyError(@()robotenergy.start(changed),'robotenergy:PackChanged');
   end
   b.use_pack_sizing=true;
   tc.verifyError(@()robotenergy.start(b),'robotenergy:FixedPack');
  end
  function resetsKeepThePackAndClearOnlyDynamicState(tc)
   b=setup_RL_MPC(); first=initialize_mpc_replay_state(b.cfg,0);
   changed=b.cfg; changed.BATTERY.SOC_init=.25; changed.BATTERY.metric_init=25;
   low=initialize_mpc_replay_state(changed,0);
   fresh=initialize_mpc_replay_state(b.cfg,0);
   tc.verifyEqual(first.energy,fresh.energy);
   tc.verifyEqual([first.energy.ns first.energy.np first.energy.capacity], ...
       [low.energy.ns low.energy.np low.energy.capacity]);
   tc.verifyEqual(low.energy.packId,first.energy.packId);
   tc.verifyEqual(low.energy.soc,.25);
   tc.verifyEqual(b.cfg.ENERGY.integration_method,'dense_midpoint_v1');
   tc.verifyFalse(b.cfg.BATTERY.use_pack_sizing);
  end
  function historicalPackRemainsReconfigurable(tc)
   b=phase3battery.manuscriptConfig(); b.n_parallel=b.n_parallel+1;
   s=robotenergy.start(b);
   tc.verifyEqual(s.np,b.n_parallel);
   tc.verifyFalse(isfield(s,'packId'));
  end
 end
end
