function [s,b,y] = sample(s,load,dt,e,b)
[s,y]=robotenergy.advance(s,load.watts,dt);
s.voltageLimitSteps=s.voltageLimitSteps+(load.requiredV>y.packV);
s.torqueLimitSteps=s.torqueLimitSteps+(load.peakTorque>e.motorTorqueLimit);
b.metric_value=y.socPct; b.soc_pct=y.socPct; b.margin_norm=y.socPct/100;
b.pack_voltage_V=y.packV; b.pack_current_A=y.packA; b.cell_voltage_V=y.cellV;
b.cell_current_A=y.cellA; b.power_W=y.watts; b.charge_As=s.chargeAs;
b.energy_Wh=s.energyWh; b.n_series=s.ns; b.n_parallel=s.np;
b.drive_required_V=load.requiredV; b.drive_voltage_leg=load.voltageLeg;
b.drive_voltage_axis=load.voltageAxis; b.drive_voltage_fsm=load.voltageFSM;
b.peak_motor_torque_Nm=load.peakTorque;
b.voltage_limit_steps=s.voltageLimitSteps; b.torque_limit_steps=s.torqueLimitSteps;
end
