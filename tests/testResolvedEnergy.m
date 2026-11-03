classdef testResolvedEnergy < matlab.unittest.TestCase
 methods(Test)
  function resolvedSequenceConservesAndEstimatorTracksTruth(tc)
   [s,e,b]=fixture(); initial=s; d=syntheticDemand([4;90;15;130],e);
   [a,b,y,~,detail]=robotenergy.sampleResolved(s,d,e,b);
   tc.verifyEqual(a.energyWh,d.fine_energy_Wh,'AbsTol',1e-14);
   tc.verifyEqual(a.soc,initial.soc-a.chargeAs/(3600*a.np*a.capacity),'AbsTol',1e-14);
   tc.verifyEqual(y.packA,a.chargeAs/d.dt,'AbsTol',1e-13);
   tc.verifyEqual(a.estimate,[a.soc;a.rc],'AbsTol',1e-14);
   tc.verifyEqual(detail.endpoint_innovation_V,0,'AbsTol',1e-13);
   tc.verifyEqual(b.drive_interval_energy_Wh,a.energyWh,'AbsTol',1e-14);
   tc.verifyEqual(a.elapsed,d.dt,'AbsTol',1e-15);
   tc.verifyGreaterThanOrEqual(min(eig(a.cov)),-1e-14);
   tc.verifyEqual(detail.prior_measurement_covariance, ...
       detail.composed_transition_jacobian*initial.cov*detail.composed_transition_jacobian'+initial.Q,'AbsTol',1e-15);
  end
  function composedJacobianMatchesFiniteDifferences(tc)
   [s,e,b]=fixture(); d=syntheticDemand([4;90;15;130],e);
   [~,~,~,~,detail]=robotenergy.sampleResolved(s,d,e,b);
   z=[.923;.02;-.01;.03]; h=1e-7; currents=detail.halfstep_cell_current_A;
   analytic=eye(4); predicted=z;
   for k=1:numel(currents)
    [predicted,~,F]=robotenergy.linearizeEstimator(s,predicted,currents(k),d.dt/numel(currents));
    analytic=F*analytic;
   end
   numerical=zeros(4);
   for j=1:4
    delta=zeros(4,1); delta(j)=h;
    numerical(:,j)=(predict(s,z+delta,currents,d.dt)-predict(s,z-delta,currents,d.dt))/(2*h);
   end
   tc.verifyEqual(analytic,numerical,'AbsTol',3e-8);
  end
  function endpointLimitsReceiveNoDurationWeight(tc)
   [s,e,b]=fixture(); d=syntheticDemand(20*ones(4,1),e);
   d.endpoint_watts=[300;200]; d.sampled_peak_voltage_V=100;
   d.peak_voltage_load.requiredV=100;
   [a,b,y,~]=robotenergy.sampleResolved(s,d,e,b);
   tc.verifyEqual(a.energyWh,20*d.dt/3600,'AbsTol',1e-14);
   tc.verifyEqual(y.watts,20,'AbsTol',1e-12);
   tc.verifyEqual(a.voltageLimitSteps,1);
   tc.verifyLessThan(b.drive_voltage_headroom_V,0);
   tc.verifyGreaterThan(b.drive_sampled_peak_cell_current_A,y.cellA);
  end
  function resolvedDerivativeMatchesIKWithRotationDrift(tc)
   bundle=setup_RL_MPC(); state=initialize_mpc_replay_state(bundle.cfg,0); p=get_params(0);
   addpath(fileparts(fileparts(mfilename('fullpath'))));
   x=state.Xt; R=reshape(x(7:15),3,3)*diag([1+1e-4,1-2e-4,1+3e-4]); x(7:15)=R(:);
   dx=zeros(30,1); dx(1:3)=[.02;-.01;.015];
   dx(7:15)=reshape(R*hatMap([.04;-.03;.02]),9,1); dx(19:30)=repmat([.006;.002;-.003],4,1);
   m=robotenergy.instantaneousLoad(x,dx,zeros(12,1),p,0,struct('FSM',ones(4,1)), ...
       state.knee_template,state.knee_parameters.Tmc,bundle.cfg.ENERGY);
   h=1e-7; numerical=zeros(3,4);
   for j=1:4
    p.sign_L=sign(p.pf34(1,j)); p.sign_d=sign(p.pf34(2,j)); idx=18+(3*j-2:3*j);
    xp=x+h*dx; xm=x-h*dx; qp=fcn_invKin3(xp,xp(idx),p)'; qm=fcn_invKin3(xm,xm(idx),p)';
    numerical(:,j)=atan2(sin(qp-qm),cos(qp-qm))/(2*h);
   end
   tc.verifyEqual(m.jointSpeed,numerical,'AbsTol',1e-8);
  end
  function unknownGridAndLegacyEstimatorRejected(tc)
   [s,e,b]=fixture(); d=syntheticDemand(20*ones(4,1),e);
   tc.verifyError(@()robotenergy.advance(s,20,.01),'robotenergy:EstimatorVersion');
   s.estimatorVersion='fixed_pack_exact_jacobian_ekf_v2';
   tc.verifyError(@()robotenergy.sampleResolved(s,d,e,b),'robotenergy:EstimatorVersion');
   s=rmfield(s,'estimatorVersion');
   tc.verifyError(@()robotenergy.sampleResolved(s,d,e,b),'robotenergy:EstimatorVersion');
   e.integration_method='unknown'; tc.verifyError(@()robotenergy.usesResolved(e),'robotenergy:IntegrationMethod');
  end
  function estimatorVersionsAreExplicitAndHistoricalDefaultsRemain(tc)
   tc.verifyEqual(robotenergy.estimatorVersion(struct()),'fixed_pack_streaming_3rc_ekf_v1');
   b=phase3battery.manuscriptConfig();
   tc.verifyEqual(robotenergy.estimatorVersion(b),'fixed_pack_exact_jacobian_ekf_v2');
   b.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
   tc.verifyEqual(robotenergy.estimatorVersion(b),'fixed_pack_resolved_ekf_v3');
   tc.verifyEqual(robotenergy.start(b).estimatorVersion,'fixed_pack_resolved_ekf_v3');
   b.streaming_estimator_version='unrecognized';
   tc.verifyError(@()robotenergy.estimatorVersion(b),'robotenergy:EstimatorVersion');
  end
  function configuredMissionReserveIsAHardPhysicalGate(tc)
   [s,e,b]=fixture(); d=syntheticDemand(20*ones(4,1),e); s.soc=e.minimum_soc;
   s.estimate(1)=s.soc;
   tc.verifyError(@()robotenergy.sampleResolved(s,d,e,b),'robotenergy:Empty');
  end
  function denseProviderPreservesMotionAndBothConsumersAgree(tc)
   b=setup_RL_MPC(); c=b.cfg;
   addpath(fileparts(fileparts(mfilename('fullpath'))));
   c.BATTERY.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
   c.ENERGY.integration_method='dense_midpoint_v1'; c.ENERGY.integration_substeps=32;
   c.ENERGY.comparison_substeps=16; c.ENERGY.minimum_soc=c.BATTERY.terminal_margin;
   c.CHUNK_DURATION=.03;
   folder=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
   c.RUN.sim_snapshot_file=fullfile(folder.Folder,'sim.mat');
   s=initialize_mpc_replay_state(c,0);
   addpath(fileparts(fileparts(mfilename('fullpath'))));
   control=struct('R',s.R,'v_cmd',.5,'a_cmd',.5,'reference_mode','position_continuous_v1');
   options=struct('duration_s',.03,'update_battery',true,'update_proxy',false,'capture_drive_trace',true);
   [dense,out]=simulate_mpc_horizon(s,control,c,options);
   tc.assertTrue(out.completed_horizon);
   legacy=c; legacy.ENERGY=robotenergy.config();
   legacy.BATTERY.streaming_estimator_version='fixed_pack_exact_jacobian_ekf_v2';
   oldState=initialize_mpc_replay_state(legacy,0);
   [old,~]=simulate_mpc_horizon(oldState,control,legacy,options);
   tc.verifyEqual(dense.Xt,old.Xt,'AbsTol',1e-13); tc.verifyEqual(dense.Ut,old.Ut,'AbsTol',1e-13);
   tc.verifyEqual(sum(out.drive_trace(:,3).*(out.drive_trace(:,2)-out.drive_trace(:,1)))/3600, ...
       dense.energy.energyWh,'AbsTol',1e-14);
   tc.verifyEqual(dense.energy.estimate,[dense.energy.soc;dense.energy.rc],'AbsTol',1e-13);
   reset_mpc_case_state(); native=run_MPC_simulation(repmat(s.R,4,1),0,.5,.5,c);
   tc.assertTrue(native.feasible); saved=load(c.RUN.sim_snapshot_file,'Sim');
   tc.verifyEqual(saved.Sim.Xt,dense.Xt,'AbsTol',1e-12);
   tc.verifyEqual(saved.Sim.energy.soc,dense.energy.soc,'AbsTol',1e-13);
   tc.verifyEqual(saved.Sim.energy.energyWh,dense.energy.energyWh,'AbsTol',1e-13);
   tc.verifyEqual(saved.Sim.energy.chargeAs,dense.energy.chargeAs,'AbsTol',1e-12);
   tc.verifyEqual(saved.Sim.energy.cov,dense.energy.cov,'AbsTol',1e-12);
  end
  function denseFailureRollsBackAcceptedInterval(tc)
   b=setup_RL_MPC(); c=b.cfg;
   addpath(fileparts(fileparts(mfilename('fullpath'))));
   c.BATTERY.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
   c.ENERGY.integration_method='dense_midpoint_v1'; c.ENERGY.integration_substeps=32;
   c.ENERGY.comparison_substeps=16; c.ENERGY.minimum_soc=c.BATTERY.terminal_margin;
   c.ENERGY.auxW=1e8; s=initialize_mpc_replay_state(c,0);
   addpath(fileparts(fileparts(mfilename('fullpath'))));
   control=struct('R',s.R,'v_cmd',.5,'a_cmd',.5,'reference_mode','position_continuous_v1');
   [after,out]=simulate_mpc_horizon(s,control,c,struct('duration_s',.03,'update_battery',true));
   tc.verifyEqual(out.terminal_reason,"battery_power_limit");
   tc.verifyEqual(after.energy,s.energy); tc.verifyEqual(after.t,s.t); tc.verifyEqual(after.Xt,s.Xt);
   tc.verifyEqual(out.charge_As,0); tc.verifyEqual(out.energy_Wh,0);
   tc.verifyGreaterThan(out.failure_solver.energy_exception.load.watts,1e8);
  end
 end
end
function [s,e,b]=fixture()
b=phase3battery.manuscriptConfig(); b.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
s=robotenergy.start(b); e=robotenergy.config();
e.integration_method='dense_midpoint_v1'; e.integration_substeps=4; e.comparison_substeps=2;
e.minimum_soc=b.terminal_margin;
end
function d=syntheticDemand(P,e)
load=struct('requiredV',1,'voltageLeg',1,'voltageAxis',1,'voltageFSM',1,'sample_time_s',0, ...
 'jointSpeed',zeros(3,4),'jointTorque',zeros(3,4),'motorSpeed',zeros(3,4), ...
 'motorTorque',zeros(3,4),'phaseCurrent',zeros(3,4),'motorVoltage',zeros(3,4));
d=struct('method',e.integration_method,'substeps',numel(P),'dt',.01,'fine_watts',P, ...
 'comparison_watts',[P(1);P(3)],'endpoint_watts',[P(1);P(end)], ...
 'sampled_peak_voltage_V',1,'sampled_peak_torque_Nm',0,'peak_voltage_load',load, ...
 'fine_energy_Wh',mean(P)*.01/3600,'comparison_energy_Wh',mean(P([1,3]))*.01/3600);
end
function z=predict(s,z,currents,dt)
for k=1:numel(currents)
 z=robotenergy.linearizeEstimator(s,z,currents(k),dt/numel(currents));
end
end
