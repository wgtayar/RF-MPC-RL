function report=run_pack_sizing_freeze(inputRoot,outputRoot)
%RUN_PACK_SIZING_FREEZE Select one fixed simulation pack from resolved demand.
% Series count is the conservative RC-envelope choice. Parallel counts are
% checked in ascending order. P95 <= characterization amplitude is the sizing
% criterion; all sampled voltage, power-root, motor-torque and SOC checks pass.
% Above-characterization current tails are allowed extrapolation, not a rating.
src=bootstrap_RF_MPC_RL(); sourceSha=gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'packFreeze:Dirty','Commit source before final analysis.');
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'packFreeze:Exists','Use a new output directory.');
inputManifest=jsondecode(fileread(fullfile(inputRoot,'manifest.json')));
inputResult=jsondecode(fileread(fullfile(inputRoot,'result.json')));
assert(inputResult.full_trace_analyzed,'packFreeze:Partial','Final sizing requires the complete requested mission.');
sampleFile=fullfile(inputRoot,inputResult.sample_file);
assert(strcmp(sha256_file(sampleFile),inputResult.sample_file_sha256),'packFreeze:Hash','Instantaneous demand file hash differs.');
configurationFile=fullfile(inputRoot,'configuration.mat');
assert(strcmp(sha256_file(configurationFile),inputManifest.input_configuration_sha256), ...
    'packFreeze:ConfigurationHash','Configuration differs from the resolved-demand input.');
loaded=load(configurationFile,'b'); b=loaded.b;
store=matfile(sampleFile); d.watts=store.watts; d.requiredV=store.required_voltage_V;
d.times=store.sample_time_s; motorTorque=store.motor_torque_peak_Nm;
d.endP=store.endpoint_watts; d.endV=store.endpoint_required_voltage_V;
endTorque=store.endpoint_motor_torque_peak_Nm; stepSummary=store.step_summary;
d.coarseP=store.comparison_watts; d.stepPeakV=stepSummary(:,4);
d.fine=inputManifest.options.substeps; d.coarse=inputManifest.options.comparison_substeps;
d.dt=inputManifest.midpoint_duration_s; d.steps=inputResult.processed_steps; N=d.steps*d.fine;
validateattributes([d.fine d.coarse d.steps],{'numeric'},{'integer','positive','finite'});
validateattributes(d.dt,{'numeric'},{'scalar','real','positive','finite'});
assert(d.fine==2*d.coarse,'packFreeze:Grid','Require coarse midpoints at odd fine-interval boundaries.');
assert(numel(d.watts)==N && numel(d.requiredV)==N && numel(d.times)==N && ...
    numel(d.coarseP)==d.steps*d.coarse && isequal(size(d.endP),[d.steps,2]) && isequal(size(d.endV),[d.steps,2]), ...
    'packFreeze:Shape','Demand dimensions differ.');
assert(all(isfinite([d.watts;d.requiredV;d.coarseP;d.endP(:);d.endV(:);d.stepPeakV])) && ...
    all([d.watts;d.requiredV;d.coarseP;d.endP(:);d.endV(:);d.stepPeakV]>=0), ...
    'packFreeze:Demand','Require finite nonnegative power and required voltage.');
expectedTimes=((1:N)'-.5)*d.dt;
tolerance=64*eps(max(1,N*d.dt));
assert(iscolumn(d.times) && all(isfinite(d.times)) && max(abs(d.times-expectedTimes))<=tolerance && ...
    abs(inputResult.duration_s-N*d.dt)<=tolerance && inputResult.input_steps==d.steps && ...
    inputResult.fine.samples==N, ...
    'packFreeze:Timeline','Require complete contiguous midpoint times from zero with matching duration and sample count.');
energy=sum(d.watts)*d.dt/3600;
peakV=max([d.requiredV;d.endV(:);d.stepPeakV]); peakP=max([d.watts;d.coarseP;d.endP(:);stepSummary(:,5)]);
peakTau=max([motorTorque;endTorque(:);stepSummary(:,6)]);
bound=robotenergy.conservativePackBound(b.cfg.BATTERY,peakV,peakP,energy,.25);
mkdir(outputRoot);
meta=struct('schema','fixed_simulation_pack_p95_instantaneous_v2','source_sha',sourceSha,'source_dirty',false, ...
    'source_branch',gitValue(src,'branch --show-current'), ...
    'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'input_manifest_sha256',sha256_file(fullfile(inputRoot,'manifest.json')), ...
    'input_result_sha256',sha256_file(fullfile(inputRoot,'result.json')), ...
    'input_configuration_sha256',sha256_file(fullfile(inputRoot,'configuration.mat')), ...
    'input_sample_file_sha256',inputResult.sample_file_sha256,'input_source_sha',inputResult.source_sha, ...
    'matlab',version,'seed',b.cfg.RNG_SEED,'reward_version',b.cfg.REWARD.version, ...
    'training',false,'episodes_trained',0,'learning_rates_used',[],'exploration_noise_used',[], ...
    'plant_step_s',d.dt/2,'power_sample_duration_s',d.dt,'estimator_updates',0, ...
    'endpoint_semantics','Zero-duration algebraic checks, with no state/charge/energy update.', ...
    'voltage_check','Each MPC interval: minimum checked loaded bus >= maximum required voltage over fine/coarse midpoints and both endpoints. This noncoincident envelope is conservative; direct fine/endpoint pointwise headrooms are also retained.', ...
    'current_semantics','P95 cell current <= 1 A characterization amplitude; tails explicitly allowed under existing extrapolation policy. 1 A is not a hardware rating.', ...
    'selection_rule','Fix Ns from the sufficient all-amplitude RC bound, then select smallest Np that passes both 25% and 95% starting SOC. Evaluate lower SOC first. No global series/parallel optimum claim.', ...
    'scope','New fixed simulated pack with ideal drive; no physical 24 V Mini-Cheetah compatibility claim.');
names={'run_pack_sizing_freeze','robotenergy.advancePlant','robotenergy.instantaneousPack','robotenergy.conservativePackBound','robotenergy.lookup'};
meta.analysis_files=cellfun(@which,names,'UniformOutput',false);
meta.analysis_file_hashes=cellfun(@sha256_file,meta.analysis_files,'UniformOutput',false);
writeJson(outputRoot,'manifest',meta); writeJson(outputRoot,'all_amplitude_reference_bound',bound);
try
    assert(peakTau<=b.cfg.ENERGY.motorTorqueLimit,'packFreeze:MotorTorque','Pack changes cannot repair motor torque exceedance.');
    candidates={}; selected=[];
    for np=1:bound.n_parallel
        candidate=struct('n_series',bound.n_series,'n_parallel',np,'cases',{{}},'qualified',false);
        % Even ideal no-sag OCV cannot spend less charge than E/(Ns*OCVmax).
        bestReserve=bound.n_series*np*bound.ocv_max_V*b.cfg.BATTERY.C_nom_Ah*(.25-b.cfg.BATTERY.terminal_margin);
        if energy>bestReserve
            candidate.status='not_replayed_necessary_reserve_bound'; candidate.maximum_ideal_reserve_Wh=bestReserve;
        else
            candidate.cases{1}=replay(b,.25,bound,np,d,outputRoot);
            if candidate.cases{1}.qualified
                candidate.cases{2}=replay(b,.95,bound,np,d,outputRoot);
            end
            candidate.qualified=numel(candidate.cases)==2 && all(cellfun(@(c)c.qualified,candidate.cases));
            candidate.status='replayed';
        end
        candidates{end+1}=candidate; %#ok<AGROW>
        writeJson(outputRoot,sprintf('candidate_%dS%dP',bound.n_series,np),candidate);
        if candidate.qualified, selected=candidate; break; end
    end
    assert(~isempty(selected),'packFreeze:NoCandidate','No fixed design passed the declared finite candidate policy.');
    report=struct('schema',meta.schema,'source_sha',sourceSha,'source_dirty',false, ...
        'all_amplitude_reference_bound',bound,'selected',selected,'candidates',{candidates}, ...
        'torque_phase_statistics',inputResult.fine,'power_quadrature_relative_grid_difference',inputResult.energy_relative_grid_difference, ...
        'sampled_peak_motor_torque_Nm',peakTau,'motor_torque_limit_Nm',b.cfg.ENERGY.motorTorqueLimit, ...
        'fixed_simulation_specification_selected',true,'runtime_configuration_modified',false, ...
        'sampled_instantaneous_electrical_qualified',true,'continuous_peak_upper_bound_proven',false, ...
        'training_ready',false,'physical_hardware_qualified',false, ...
        'required_full_charge_bus_capability_V',bound.required_full_charge_bus_capability_V, ...
        'required_drive_capability','Ideal simulated drive must support full-charge pack voltage; real driver, motor speed/insulation, protection, thermal and mass validation remain separate.', ...
        'limitations','Sampled voltage checks use a conservative within-interval envelope. Demand maxima are not a continuous upper bound or a complete future RL action-domain qualification. Selected pack remains identical across every RL episode.');
    assert(strcmp(sourceSha,gitValue(src,'rev-parse HEAD')) && isempty(gitValue(src,'status --porcelain')), ...
        'packFreeze:SourceChanged','Source changed during final sizing.');
    writeJson(outputRoot,'selection',report);
catch exception
    writeJson(outputRoot,'failure',struct('source_sha',sourceSha,'identifier',exception.identifier, ...
        'message',exception.message,'stack',{exception.stack}));
    rethrow(exception)
end
end

function r=replay(bundle,initialSoc,bound,np,d,outputRoot)
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
function value=gitValue(root,args)
[status,value]=system(sprintf('git -C "%s" %s',root,args)); assert(status==0); value=strtrim(value);
end
function writeJson(root,name,value)
fid=fopen(fullfile(root,[name '.json']),'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
