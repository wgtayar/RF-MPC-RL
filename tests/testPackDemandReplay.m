classdef testPackDemandReplay < matlab.unittest.TestCase
 methods(Test)
  function booksPhysicalPowerAndChargeOnce(tc)
   [b,reference,d,root]=fixture(tc,10,10);
   r=robotenergy.replayDemand(b,.95,reference,1,d,root);
   tc.verifyTrue(r.completed_full_trace);
   tc.verifyTrue(r.qualified);
   tc.verifyEqual(r.energy_Wh,sum(d.watts)*d.dt/3600,'AbsTol',1e-12);
   tc.verifyEqual(r.final_true_soc,.95-r.charge_As/(3600*2),'AbsTol',1e-12);
   tc.verifyEqual(r.duration_completed_s,numel(d.watts)*d.dt,'AbsTol',1e-12);
   archive=load(fullfile(root,r.file),'midpoint_trace','valid_midpoint_rows', ...
       'valid_interval_rows','final_state','b');
   tc.verifyEqual(archive.valid_midpoint_rows,numel(d.watts));
   tc.verifyEqual(archive.valid_interval_rows,d.steps);
   tc.verifyEqual(r.cell_current_p95_A,max(archive.midpoint_trace(:,2)));
   tc.verifyEqual(archive.final_state.estimate,[.95;0;0;0]);
   tc.verifyFalse(isfield(archive.b,'fixed_pack_spec'));
  end
  function firstVoltageFailureHasNoMissionPercentile(tc)
   [b,reference,d,root]=fixture(tc,10,1000);
   r=robotenergy.replayDemand(b,.95,reference,1,d,root);
   tc.verifyFalse(r.completed_full_trace);
   tc.verifyFalse(r.qualified);
   tc.verifyEqual(r.samples_completed,0);
   tc.verifyEqual(r.duration_completed_s,0);
   tc.verifyTrue(isnan(r.cell_current_p95_A));
   tc.verifyEqual(r.failure.identifier,'packFreeze:VoltageEnvelope');
   tc.verifyLessThan(r.minimum_conservative_interval_voltage_headroom_V,0);
  end
  function partialFailurePreservesAttemptWithoutClaimingCompleteRows(tc)
   [b,reference,d,root]=fixture(tc,1000,78);
   d.endP(:)=0;
   r=robotenergy.replayDemand(b,.95,reference,1,d,root);
   tc.verifyFalse(r.completed_full_trace);
   tc.verifyEqual(r.samples_completed,0);
   tc.verifyEqual(r.duration_completed_s,d.dt/2,'AbsTol',1e-12);
   tc.verifyGreaterThan(r.charge_As,0);
   tc.verifyGreaterThan(r.energy_Wh,0);
   tc.verifyTrue(isnan(r.cell_current_p95_A));
   tc.verifyTrue(isnan(r.cell_current_rms_A));
   tc.verifyEqual(r.failure.attempted.phase,'first_halfstep');
   tc.verifyEqual(r.failure.identifier,'packFreeze:VoltageEnvelope');
  end
 end
end
function [b,reference,d,root]=fixture(tc,power,voltage)
root=tempname; mkdir(root); tc.addTeardown(@()rmdir(root,'s'));
b=struct('cfg',struct('BATTERY',phase3battery.manuscriptConfig()));
reference=struct('n_series',20,'characterization_current_A',1);
d=struct('fine',4,'coarse',2,'dt',.0025,'steps',3);
N=d.steps*d.fine;
d.watts=repmat(power,N,1); d.requiredV=repmat(voltage,N,1);
d.times=((1:N)'-.5)*d.dt;
d.endP=repmat(power,d.steps,2); d.endV=repmat(voltage,d.steps,2);
d.coarseP=repmat(power,d.steps*d.coarse,1); d.stepPeakV=repmat(voltage,d.steps,1);
end
