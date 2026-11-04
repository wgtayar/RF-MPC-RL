function r=replayDemand(bundle,initialSoc,bound,np,d,outputRoot)
%REPLAYDEMAND Shared offline physical replay; no estimator updates or pack selection.
% Extracted without arithmetic changes from run_pack_sizing_freeze.
b=bundle.cfg.BATTERY; b.n_series=bound.n_series; b.n_parallel=np;
b.SOC_init=initialSoc; b.metric_init=100*initialSoc; b.use_pack_sizing=false;
s=robotenergy.start(b); initialEstimate=s.estimate; initialCov=s.cov;
N=numel(d.watts); blockSteps=1000;
stem=sprintf('replay_%dS%dP_soc%02d',s.ns,s.np,round(100*initialSoc)); path=fullfile(outputRoot,[stem '.mat']);
save(path,'b','bound','-v7.3'); archive=matfile(path,'Writable',true);
archive.midpoint_trace(N,8)=0; archive.endpoint_trace(d.steps,6)=0;
archive.coarse_trace(d.steps*d.coarse,4)=0; archive.interval_envelope(d.steps,4)=0;
archive.midpoint_columns={'time_s','instant_cell_A','pack_V','true_SOC_midpoint','mean_hold_current_A','pointwise_voltage_headroom_V','charge_As_after','energy_Wh_after'};
archive.endpoint_columns={'left_cell_A','left_pack_V','left_pointwise_headroom_V','right_cell_A','right_pack_V','right_pointwise_headroom_V'};
archive.coarse_columns={'time_s','instant_cell_A','pack_V','conservative_interval_headroom_V'};
archive.envelope_columns={'time_before_s','maximum_sampled_required_voltage_V','minimum_checked_loaded_bus_V','conservative_voltage_headroom_V'};
instantCurrent=zeros(N,1); maxCurrent=0; minimumBus=Inf; minimumHeadroom=Inf; minimumPointwise=Inf; minimumSOC=initialSoc;
started=tic; processed=0; completedSteps=0; failure=[]; unexpectedException=[]; attempted=struct();
firstStep=1; rows=[]; endpoint=[]; coarseRows=[]; envelopes=[]; n=0; nc=0; ne=0;
try
    for firstStep=1:blockSteps:d.steps
        lastStep=min(d.steps,firstStep+blockSteps-1); count=lastStep-firstStep+1;
        rows=zeros(count*d.fine,8); endpoint=zeros(count,6); coarseRows=zeros(count*d.coarse,4); envelopes=zeros(count,4);
        n=0; nc=0; ne=0;
        for step=firstStep:lastStep
            needed=d.stepPeakV(step); stepMinBus=Inf;
            attempted=struct('mpc_interval',step,'phase','left_endpoint','state_time_s',s.elapsed, ...
                'power_W',d.endP(step,1),'required_interval_voltage_V',needed);
            left=robotenergy.instantaneousPack(s,d.endP(step,1));
            endpoint(step-firstStep+1,1:3)=[left.cellA,left.packV,left.packV-d.endV(step,1)];
            checkPoint(left,needed); minimumPointwise=min(minimumPointwise,left.packV-d.endV(step,1));
            for j=1:d.fine
                index=(step-1)*d.fine+j; P=d.watts(index);
                attempted=struct('mpc_interval',step,'fine_sample',index,'phase','first_halfstep', ...
                    'state_time_s',s.elapsed,'power_W',P,'required_interval_voltage_V',needed);
                [s,half1]=robotenergy.advancePlant(s,P,d.dt/2); checkPoint(half1,needed);
                attempted.phase='midpoint'; attempted.state_time_s=s.elapsed;
                center=robotenergy.instantaneousPack(s,P); midSoc=s.soc; checkPoint(center,needed);
                attempted.phase='second_halfstep';
                [s,half2]=robotenergy.advancePlant(s,P,d.dt/2); checkPoint(half2,needed);
                instantCurrent(index)=center.cellA; headroom=center.packV-d.requiredV(index);
                minimumPointwise=min(minimumPointwise,headroom); processed=index;
                n=n+1; rows(n,:)=[d.times(index),center.cellA,center.packV,midSoc,(half1.cellA+half2.cellA)/2,headroom,s.chargeAs,s.energyWh];
                if mod(j,2)==1
                    cindex=(step-1)*d.coarse+(j+1)/2;
                    attempted.phase='coarse_midpoint'; attempted.state_time_s=s.elapsed; attempted.power_W=d.coarseP(cindex);
                    coarse=robotenergy.instantaneousPack(s,d.coarseP(cindex)); checkPoint(coarse,needed);
                    nc=nc+1; coarseRows(nc,:)=[d.times(index)+d.dt/2,coarse.cellA,coarse.packV,coarse.packV-needed];
                end
            end
            attempted=struct('mpc_interval',step,'phase','right_endpoint','state_time_s',s.elapsed, ...
                'power_W',d.endP(step,2),'required_interval_voltage_V',needed);
            right=robotenergy.instantaneousPack(s,d.endP(step,2)); checkPoint(right,needed);
            endpoint(step-firstStep+1,4:6)=[right.cellA,right.packV,right.packV-d.endV(step,2)];
            minimumPointwise=min(minimumPointwise,right.packV-d.endV(step,2));
            ne=ne+1; envelopes(ne,:)=[d.times((step-1)*d.fine+1)-d.dt/2,needed,stepMinBus,stepMinBus-needed];
            completedSteps=step;
        end
        saveBlock(); n=0; nc=0; ne=0;
        fprintf('Fixed pack %dS%dP SOC%.0f: %d/%d intervals, %.1f wall s\n',s.ns,s.np,initialSoc*100,lastStep,d.steps,toc(started));
    end
catch exception
    saveBlock();
    failure=struct('identifier',exception.identifier,'message',exception.message,'stack',{exception.stack}, ...
        'processed_midpoint_samples',processed,'completed_mpc_intervals',completedSteps,'elapsed_state_s',s.elapsed, ...
        'attempted',attempted,'plant_state',s);
    expected={'robotenergy:PowerLimit','robotenergy:Empty','packFreeze:Reserve','packFreeze:VoltageEnvelope'};
    if ~ismember(exception.identifier,expected), unexpectedException=exception; end
end
assert(isequaln(s.estimate,initialEstimate) && isequaln(s.cov,initialCov),'packFreeze:Estimator','Fine replay must not update EKF state or covariance.');
full=isempty(failure) && processed==N;
if full
    p95=nearest95(instantCurrent); rms=sqrt(mean(instantCurrent.^2));
    energyError=s.energyWh-sum(d.watts)*d.dt/3600;
    chargeError=s.soc-(initialSoc-s.chargeAs/(3600*s.np*s.capacity));
    assert(abs(energyError)<1e-9*max(1,s.energyWh) && abs(chargeError)<1e-9,'packFreeze:Conservation','Conservation failed beyond accumulation roundoff.');
else
    p95=NaN; rms=NaN; energyError=NaN; chargeError=NaN;
end
archive.final_state=s; archive.failure=failure; archive.valid_midpoint_rows=processed;
archive.valid_interval_rows=completedSteps; archive.completed_full_trace=full;
r=struct('initial_soc',initialSoc,'n_series',s.ns,'n_parallel',s.np,'samples_completed',processed,'samples_requested',N, ...
    'duration_completed_s',s.elapsed,'completed_full_trace',full,'cell_current_p95_A',p95,'cell_current_rms_A',rms, ...
    'maximum_checked_cell_current_A',maxCurrent,'characterization_current_A',bound.characterization_current_A, ...
    'observed_cell_current_above_characterization_duration_s',d.dt*nnz(instantCurrent(1:processed)>bound.characterization_current_A), ...
    'minimum_pack_voltage_V',minimumBus,'minimum_conservative_interval_voltage_headroom_V',minimumHeadroom, ...
    'minimum_direct_pointwise_voltage_headroom_V',minimumPointwise,'minimum_true_soc',minimumSOC,'final_true_soc',s.soc, ...
    'mission_soc_floor',b.terminal_margin,'charge_As',s.chargeAs,'energy_Wh',s.energyWh, ...
    'energy_accumulation_error_Wh',energyError,'charge_soc_accumulation_error',chargeError,'estimator_updates',0, ...
    'sampled_electrical_qualified',full,'p95_current_supported',full && p95<=bound.characterization_current_A, ...
    'qualified',full && p95<=bound.characterization_current_A,'failure',failure,'file',[stem '.mat'], ...
    'file_sha256',sha256_file(path),'wall_s',toc(started), ...
    'current_statistic','P95/RMS at equally weighted fine quadrature midpoints; peak includes half-steps, coarse midpoints and endpoints. Incomplete traces have no mission P95.', ...
    'tail_policy','Current above the characterization amplitude is explicitly extrapolated; no hardware current rating is implied.');
writeJson(outputRoot,stem,r);
if ~isempty(unexpectedException), rethrow(unexpectedException); end
    function checkPoint(y,needed)
        attempted.evaluated_cell_current_A=y.cellA; attempted.evaluated_pack_voltage_V=y.packV; attempted.evaluated_soc=s.soc;
        maxCurrent=max(maxCurrent,y.cellA); minimumBus=min(minimumBus,y.packV); stepMinBus=min(stepMinBus,y.packV);
        minimumHeadroom=min(minimumHeadroom,y.packV-needed); minimumSOC=min(minimumSOC,s.soc);
        assert(s.soc>=b.terminal_margin,'packFreeze:Reserve','True SOC falls below the mission reserve.');
        assert(y.packV>=needed,'packFreeze:VoltageEnvelope','Loaded bus fails the conservative within-MPC-interval voltage envelope.');
    end
    function saveBlock()
        if n>0, archive.midpoint_trace((firstStep-1)*d.fine+(1:n),1:8)=rows(1:n,:); end
        if nc>0, archive.coarse_trace((firstStep-1)*d.coarse+(1:nc),1:4)=coarseRows(1:nc,:); end
        if ne>0
            archive.endpoint_trace(firstStep:firstStep+ne-1,1:6)=endpoint(1:ne,:);
            archive.interval_envelope(firstStep:firstStep+ne-1,1:4)=envelopes(1:ne,:);
        end
    end
end
function value=nearest95(x)
x=sort(x); value=x(ceil(.95*numel(x)));
end
function writeJson(root,name,value)
fid=fopen(fullfile(root,[name '.json']),'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
