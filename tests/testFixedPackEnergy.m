classdef testFixedPackEnergy < matlab.unittest.TestCase
 methods(Test)
 function conservation(tc)
 b=phase3battery.manuscriptConfig(); s=robotenergy.start(b); a=s;
 [s,y]=robotenergy.advance(s,120,.1);
 tc.verifyEqual(y.packA*y.packV,120,'AbsTol',1e-11);
 tc.verifyEqual(s.energyWh,120*.1/3600,'AbsTol',1e-14);
 tc.verifyEqual(a.soc-s.soc,s.chargeAs/(3600*s.capacity*s.np),'AbsTol',1e-14);
 tc.verifyEqual(y.cellA*6,y.packA); tc.verifyEqual(y.cellV*4,y.packV);
 tc.verifyError(@() robotenergy.advance(a,1e8,.1),'robotenergy:PowerLimit');
 b.n_series=8; [~,y2]=robotenergy.advance(robotenergy.start(b),120,.1);
 tc.verifyLessThan(y2.packA,y.packA/2);
 b.use_pack_sizing=true; tc.verifyError(@()robotenergy.start(b),'robotenergy:FixedPack');
 end
 function staticLoad(tc)
 bundle=setup_RL_MPC('legacy_frozen_v1'); c=bundle.cfg; s=initialize_mpc_replay_state(c,0); p=get_params(0);
 f=struct('FSM',ones(4,1)); forces=repmat([0;0;p.mass*p.g/4],4,1);
 evalLoad=@(U)robotenergy.loadStep(s.Xt,s.Xt,U,p,.01,0,f,s.knee_template,s.knee_parameters.Tmc,c.ENERGY);
 zero=evalLoad(0*forces); hold=evalLoad(forces); heavy=evalLoad(2*forces);
 tc.verifyEqual(zero.watts,0); tc.verifyGreaterThan(hold.watts,0);
 tc.verifyEqual(heavy.watts,4*hold.watts,'RelTol',1e-10);
 tc.verifyEqual(hold.copperW,hold.watts,'AbsTol',1e-12);
 end
 function virtualWork(tc)
 bundle=setup_RL_MPC('legacy_frozen_v1'); c=bundle.cfg; s=initialize_mpc_replay_state(c,0); p=get_params(0);
 f=struct('FSM',ones(4,1)); F=repmat([2;-1;13],4,1);
 dx=[.001;-.002;.003]; x1=s.Xt; dt=1e-6;
 x1(19:30)=x1(19:30)+repmat(dx,4,1)*dt;
 m=robotenergy.loadStep(s.Xt,x1,F,p,dt,0,f,s.knee_template,s.knee_parameters.Tmc,c.ENERGY);
 tc.verifyEqual(sum(m.jointTorque.*m.jointSpeed,'all'),-F'*repmat(dx,4,1),'AbsTol',1e-6);
 end
 function chunkResume(tc)
 b=setup_RL_MPC('legacy_frozen_v1'); c=b.cfg; s=initialize_mpc_replay_state(c,0);
 ctl=struct('R',s.R,'v_cmd',.5,'a_cmd',.5,'reference_mode','position_continuous_v1','integration_time_origin_s',0);
 opt=struct('duration_s',.10,'capture_trace',false,'update_proxy',false,'update_battery',true);
 [a,all]=simulate_mpc_horizon(s,ctl,c,opt);
 opt.duration_s=.05; [z,left]=simulate_mpc_horizon(s,ctl,c,opt);
 [z,right]=simulate_mpc_horizon(z,ctl,c,opt);
 tc.verifyEqual(a.energy,z.energy); tc.verifyEqual(all.charge_As,left.charge_As+right.charge_As,'AbsTol',1e-12);
 tc.verifyEqual(a.battery.soc_pct,100*a.energy.estimate(1));
 end
 end
end
