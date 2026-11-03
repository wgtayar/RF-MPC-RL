function demand=loadResolved(solution,derivative,U,p,fsm,tpl,Tmc,e)
%LOADRESOLVED Time-resolved load on the original ODE dense solution.
% Uniform midpoints carry duration weight; endpoints and the independent
% half-size midpoint grid contribute only to sampled hard-limit checks.
assert(robotenergy.usesResolved(e),'robotenergy:IntegrationMethod','Resolved integration is not enabled.');
n=e.integration_substeps; dt=solution.x(end)-solution.x(1); t0=solution.x(1);
fine=2:2:2*n; coarse=3:4:2*n-1;
selected=unique([1,fine,coarse,2*n+1]);
times=t0+(0:2*n)*dt/(2*n); X=deval(solution,times);
watts=zeros(2*n+1,1); voltage=watts; torque=watts;
peakLoad=struct('requiredV',-inf);
for k=selected
    x=X(:,k); m=robotenergy.instantaneousLoad(x,derivative(times(k),x), ...
        U,p,times(k),fsm,tpl,Tmc,e);
    watts(k)=m.watts; voltage(k)=m.requiredV; torque(k)=m.peakTorque;
    if m.requiredV>peakLoad.requiredV
        peakLoad=m; peakLoad.sample_time_s=times(k);
    end
end
demand=struct('method',e.integration_method,'dt',dt,'time_before_s',t0, ...
    'substeps',n,'fine_time_s',times(fine)','fine_watts',watts(fine), ...
    'fine_required_V',voltage(fine),'fine_motor_torque_Nm',torque(fine), ...
    'comparison_watts',watts(coarse),'comparison_required_V',voltage(coarse), ...
    'endpoint_watts',watts([1,end]),'endpoint_required_V',voltage([1,end]), ...
    'sampled_peak_torque_Nm',max(torque(selected)), ...
    'sampled_peak_voltage_V',max(voltage(selected)),'peak_voltage_load',peakLoad, ...
    'fine_energy_Wh',mean(watts(fine))*dt/3600, ...
    'comparison_energy_Wh',mean(watts(coarse))*dt/3600);
demand.watts=mean(watts(fine));
demand.requiredV=demand.sampled_peak_voltage_V;
demand.peakTorque=demand.sampled_peak_torque_Nm;
end
