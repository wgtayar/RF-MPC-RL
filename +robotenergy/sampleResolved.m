function [s,b,y,load,detail]=sampleResolved(s,demand,e,b)
%SAMPLERESOLVED Resolve physical demand; one EKF observation per MPC step.
% Plant evolution uses two constant-power halfsteps per fine midpoint. The
% estimator composes exact local transitions at the measured halfstep
% currents. Q and the voltage measurement are applied ONCE at interval end,
% preserving the original 100 Hz noise-covariance interpretation.
% Both endpoint and comparison-grid powers are zero-duration load solves.
% Limit headroom uses min(sampled bus V)-max(sampled required V) over the
% interval: a conservative noncoincident envelope, not a peak certificate.
assert(robotenergy.usesResolved(e),'robotenergy:IntegrationMethod','Resolved integration is not enabled.');
assert(isfield(s,'estimatorVersion') && strcmp(s.estimatorVersion, ...
    'fixed_pack_resolved_ekf_v3'), 'robotenergy:EstimatorVersion', ...
    'Resolved integration requires the explicitly versioned resolved estimator.');
n=demand.substeps; dt=demand.dt; h=dt/(2*n);
validateattributes(e.minimum_soc,{'numeric'},{'scalar','real','finite','>=',s.grid(1),'<=',s.grid(end)});
assert(s.soc>=e.minimum_soc,'robotenergy:Empty','Cell SOC below configured mission reserve.');
assert(n==e.integration_substeps && numel(demand.fine_watts)==n && ...
    numel(demand.comparison_watts)==n/2, ...
    'robotenergy:IntegrationGrid','Demand does not match the configured grid.');
chargeBefore=s.chargeAs; energyBefore=s.energyWh;
z=s.estimate; Ftotal=eye(4); priorCov=s.cov;
start=robotenergy.instantaneousPack(s,demand.endpoint_watts(1));
minBus=start.packV; peakCellA=start.cellA;
halfCurrent=zeros(2*n,1); midpointCurrent=zeros(n,1); midpointBus=zeros(n,1);
for k=1:n
    P=demand.fine_watts(k);
    [s,a]=robotenergy.advancePlant(s,P,h);
    assert(s.soc>=e.minimum_soc,'robotenergy:Empty','Cell SOC below configured mission reserve.');
    [z,~,F]=robotenergy.linearizeEstimator(s,z,a.cellA,h); Ftotal=F*Ftotal;
    halfCurrent(2*k-1)=a.cellA;
    middle=robotenergy.instantaneousPack(s,P);
    midpointCurrent(k)=middle.cellA; midpointBus(k)=middle.packV;
    minBus=min([minBus,a.packV,middle.packV]); peakCellA=max([peakCellA,a.cellA,middle.cellA]);
    [s,a]=robotenergy.advancePlant(s,P,h);
    assert(s.soc>=e.minimum_soc,'robotenergy:Empty','Cell SOC below configured mission reserve.');
    [z,~,F]=robotenergy.linearizeEstimator(s,z,a.cellA,h); Ftotal=F*Ftotal;
    halfCurrent(2*k)=a.cellA;
    minBus=min(minBus,a.packV); peakCellA=max(peakCellA,a.cellA);
    if mod(k,2)==1
        comparison=robotenergy.instantaneousPack(s,demand.comparison_watts((k+1)/2));
        minBus=min(minBus,comparison.packV); peakCellA=max(peakCellA,comparison.cellA);
    end
end
last=robotenergy.instantaneousPack(s,demand.endpoint_watts(2));
minBus=min(minBus,last.packV); peakCellA=max(peakCellA,last.cellA);
% The final observation is instantaneous at the final SOC/RC state. Unlike
% advance(), it must not reconstruct a prior frozen-parameter SOC.
[r,ocv,dr,docv]=robotenergy.lookup(s,z(1));
predictedV=ocv-r(1)*last.cellA-sum(z(2:4)); H=[docv-dr(1)*last.cellA,-1,-1,-1];
C=Ftotal*priorCov*Ftotal'+s.Q;
K=C*H'/(H*C*H'+s.Rv); innovation=last.cellV-predictedV;
z=z+K*innovation; z(1)=max(s.grid(1),min(s.grid(end),z(1))); s.estimate=z;
M=eye(4)-K*H; s.cov=M*C*M'+K*s.Rv*K';
load=demand.peak_voltage_load;
load.watts=(s.energyWh-energyBefore)*3600/dt;
load.requiredV=demand.sampled_peak_voltage_V;
load.peakTorque=demand.sampled_peak_torque_Nm;
headroom=minBus-load.requiredV;
s.voltageLimitSteps=s.voltageLimitSteps+(headroom<0);
s.torqueLimitSteps=s.torqueLimitSteps+(load.peakTorque>e.motorTorqueLimit);
% Scalar current is the charge-equivalent interval average. Voltage is the
% actual terminal endpoint measurement; their product is not mean power.
meanA=(s.chargeAs-chargeBefore)/dt;
y=struct('packA',meanA,'cellA',meanA/s.np,'packV',last.packV, ...
    'cellV',last.cellV,'watts',load.watts,'socPct',100*z(1), ...
    'endpoint_packA',last.packA,'endpoint_cellA',last.cellA);
b.metric_value=y.socPct; b.soc_pct=y.socPct; b.margin_norm=y.socPct/100;
b.pack_voltage_V=y.packV; b.pack_current_A=y.packA; b.cell_voltage_V=y.cellV;
b.cell_current_A=y.cellA; b.power_W=y.watts; b.charge_As=s.chargeAs;
b.energy_Wh=s.energyWh; b.n_series=s.ns; b.n_parallel=s.np;
b.drive_required_V=load.requiredV; b.drive_voltage_leg=load.voltageLeg;
b.drive_voltage_axis=load.voltageAxis; b.drive_voltage_fsm=load.voltageFSM;
b.peak_motor_torque_Nm=load.peakTorque;
b.voltage_limit_steps=s.voltageLimitSteps; b.torque_limit_steps=s.torqueLimitSteps;
b.drive_voltage_headroom_V=headroom; b.drive_min_pack_voltage_V=minBus;
b.drive_joint_speed_rad_s=load.jointSpeed; b.drive_joint_torque_Nm=load.jointTorque;
b.drive_motor_speed_rad_s=load.motorSpeed; b.drive_motor_torque_Nm=load.motorTorque;
b.drive_phase_current_A=load.phaseCurrent; b.drive_motor_voltage_V=load.motorVoltage;
b.drive_integration_method=demand.method; b.drive_integration_substeps=n;
b.drive_peak_voltage_time_s=load.sample_time_s;
b.drive_comparison_energy_Wh=demand.comparison_energy_Wh;
b.drive_interval_energy_Wh=s.energyWh-energyBefore;
b.drive_sampled_peak_cell_current_A=peakCellA;
b.drive_current_semantics='charge-equivalent interval average; pack voltage is endpoint instantaneous';
b.drive_joint_semantics='all joint arrays at sampled maximum required-voltage time';
b.drive_headroom_semantics='minimum sampled bus voltage minus maximum sampled required voltage';
detail=struct('demand',demand,'halfstep_cell_current_A',halfCurrent, ...
    'midpoint_cell_current_A',midpointCurrent,'midpoint_pack_voltage_V',midpointBus, ...
    'min_pack_voltage_V',minBus,'peak_cell_current_A',peakCellA, ...
    'composed_transition_jacobian',Ftotal,'measurement_jacobian',H, ...
    'prior_measurement_covariance',C,'endpoint_innovation_V',innovation);
end
