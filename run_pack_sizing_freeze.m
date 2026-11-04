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
names={'run_pack_sizing_freeze','robotenergy.replayDemand','robotenergy.advancePlant','robotenergy.instantaneousPack','robotenergy.conservativePackBound','robotenergy.lookup'};
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
            candidate.cases{1}=robotenergy.replayDemand(b,.25,bound,np,d,outputRoot);
            if candidate.cases{1}.qualified
                candidate.cases{2}=robotenergy.replayDemand(b,.95,bound,np,d,outputRoot);
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

function value=gitValue(root,args)
[status,value]=system(sprintf('git -C "%s" %s',root,args)); assert(status==0); value=strtrim(value);
end
function writeJson(root,name,value)
fid=fopen(fullfile(root,[name '.json']),'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
