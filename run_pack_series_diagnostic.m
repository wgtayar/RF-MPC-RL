function report=run_pack_series_diagnostic(inputRoot,outputRoot,options)
%RUN_PACK_SERIES_DIAGNOSTIC Compare fixed series counts on the same demand.
% No pack is selected, no locked configuration is changed, and no policy is
% trained. Candidate packs use the existing 2 Ah cell, one parallel string,
% and the same resolved requested-motion trace. Battery states are fresh and
% unpolarized for each candidate/start-SOC case. Infeasible cases stop at the
% first failed sampled check and cannot report a whole-mission current P95.
if nargin<3, options=struct(); end
defaults=struct('series_counts',16:20,'initial_soc',[.95 1], ...
    'baseline_selection_root','');
names=fieldnames(options);
assert(all(ismember(names,fieldnames(defaults))),'packSeries:Options','Unknown diagnostic option.');
for k=1:numel(names), defaults.(names{k})=options.(names{k}); end
options=defaults;
validateattributes(options.series_counts,{'numeric'},{'vector','real','finite','integer','positive'});
validateattributes(options.initial_soc,{'numeric'},{'vector','real','finite','>=',.95,'<=',1});
assert(isequal(options.series_counts,unique(options.series_counts,'sorted')) && ...
    isequal(options.initial_soc,unique(options.initial_soc,'sorted')), ...
    'packSeries:Options','Counts and SOC cases must be increasing without duplicates.');
src=bootstrap_RF_MPC_RL(); sourceSha=gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'packSeries:Dirty','Commit source before the diagnostic campaign.');
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'packSeries:Exists','Use a new output directory.');
inputManifest=jsondecode(fileread(fullfile(inputRoot,'manifest.json')));
inputResult=jsondecode(fileread(fullfile(inputRoot,'result.json')));
assert(inputResult.full_trace_analyzed,'packSeries:Partial','Requires the full resolved demand trace.');
sampleFile=fullfile(inputRoot,inputResult.sample_file);
assert(strcmp(sha256_file(sampleFile),inputResult.sample_file_sha256),'packSeries:Hash','Demand hash differs.');
configurationFile=fullfile(inputRoot,'configuration.mat');
assert(strcmp(sha256_file(configurationFile),inputManifest.input_configuration_sha256), ...
    'packSeries:ConfigurationHash','Input configuration hash differs.');
loaded=load(configurationFile,'b'); bundle=loaded.b; originalBattery=bundle.cfg.BATTERY;
% Explicit diagnostic factory: do not remove a lock or mutate fixedPackConfig.
diagnosticBattery=phase3battery.manuscriptConfig(originalBattery.model_options.parameter_source);
diagnosticBattery.model_options=originalBattery.model_options;
diagnosticBattery.C_nom_Ah=originalBattery.C_nom_Ah;
diagnosticBattery.terminal_margin=originalBattery.terminal_margin;
diagnosticBattery.use_pack_sizing=false;
assert(diagnosticBattery.C_nom_Ah==2 && ~isfield(diagnosticBattery,'fixed_pack_spec'), ...
    'packSeries:Cell','This comparison uses the existing unlocked diagnostic 2 Ah cell only.');
bundle.cfg.BATTERY=diagnosticBattery;
store=matfile(sampleFile); d.watts=store.watts; d.requiredV=store.required_voltage_V;
d.times=store.sample_time_s; motorTorque=store.motor_torque_peak_Nm;
d.endP=store.endpoint_watts; d.endV=store.endpoint_required_voltage_V;
endTorque=store.endpoint_motor_torque_peak_Nm; stepSummary=store.step_summary;
d.coarseP=store.comparison_watts; d.stepPeakV=stepSummary(:,4);
d.fine=inputManifest.options.substeps; d.coarse=inputManifest.options.comparison_substeps;
d.dt=inputManifest.midpoint_duration_s; d.steps=inputResult.processed_steps; N=d.steps*d.fine;
validateattributes([d.fine d.coarse d.steps],{'numeric'},{'integer','positive','finite'});
validateattributes(d.dt,{'numeric'},{'scalar','real','positive','finite'});
assert(d.fine==2*d.coarse,'packSeries:Grid','Coarse midpoint grid does not match the replay method.');
assert(numel(d.watts)==N && numel(d.requiredV)==N && numel(d.times)==N && ...
    numel(d.coarseP)==d.steps*d.coarse && isequal(size(d.endP),[d.steps,2]) && ...
    isequal(size(d.endV),[d.steps,2]) && numel(d.stepPeakV)==d.steps, ...
    'packSeries:Shape','Demand dimensions differ.');
assert(all(isfinite([d.watts;d.requiredV;d.coarseP;d.endP(:);d.endV(:);d.stepPeakV])) && ...
    all([d.watts;d.requiredV;d.coarseP;d.endP(:);d.endV(:);d.stepPeakV]>=0), ...
    'packSeries:Demand','Require finite nonnegative demand.');
tolerance=64*eps(max(1,N*d.dt)); expectedTimes=((1:N)'-.5)*d.dt;
assert(iscolumn(d.times) && all(isfinite(d.times)) && max(abs(d.times-expectedTimes))<=tolerance && ...
    abs(inputResult.duration_s-N*d.dt)<=tolerance && inputResult.input_steps==d.steps && inputResult.fine.samples==N, ...
    'packSeries:Timeline','Requires complete contiguous resolved samples from zero.');
energy=sum(d.watts)*d.dt/3600;
peakV=max([d.requiredV;d.endV(:);d.stepPeakV]);
peakP=max([d.watts;d.coarseP;d.endP(:);stepSummary(:,5)]);
peakTau=max([motorTorque;endTorque(:);stepSummary(:,6)]);
% This supplies exact PCHIP OCV extrema and the documented 1 A reference.
% Its old series/parallel recommendation is not used to select a candidate.
reference=robotenergy.conservativePackBound(diagnosticBattery,peakV,peakP,energy,.25);
mkdir(outputRoot);
save(fullfile(outputRoot,'diagnostic_configuration.mat'),'bundle','options','reference','-v7.3');
meta=struct('schema','fixed_series_high_soc_diagnostic_v1','source_sha',sourceSha,'source_dirty',false, ...
    'source_branch',gitValue(src,'branch --show-current'), ...
    'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'input_root',inputRoot,'input_manifest_sha256',sha256_file(fullfile(inputRoot,'manifest.json')), ...
    'input_result_sha256',sha256_file(fullfile(inputRoot,'result.json')), ...
    'input_configuration_sha256',sha256_file(configurationFile), ...
    'input_sample_file_sha256',inputResult.sample_file_sha256,'input_source_sha',inputResult.source_sha, ...
    'diagnostic_configuration_sha256',sha256_file(fullfile(outputRoot,'diagnostic_configuration.mat')), ...
    'cell_parameter_file_sha256',sha256_file(diagnosticBattery.model_options.parameter_source), ...
    'matlab',version,'seed',bundle.cfg.RNG_SEED,'rng_used',false,'demand_reward_version',bundle.cfg.REWARD.version, ...
    'training',false,'episodes_trained',0,'learning_rates_used',[],'exploration_noise_used',[], ...
    'plant_step_s',d.dt/2,'power_sample_duration_s',d.dt,'estimator_updates',0,'options',options, ...
    'candidate_parallel_count',1,'candidate_cell_capacity_Ah',diagnosticBattery.C_nom_Ah, ...
    'mission_soc_floor',diagnosticBattery.terminal_margin,'required_voltage_peak_V',peakV, ...
    'voltage_check','Minimum checked loaded bus in each original MPC interval must exceed that interval''s maximum sampled required voltage. A failure stops the candidate.', ...
    'necessary_voltage_prefilter','For nonnegative discharge and unpolarized initial RC state, loaded bus cannot exceed Ns times maximum cell OCV over the declared SOC domain. This is a necessary condition only.', ...
    'current_semantics','Time-weighted P95 of fine midpoint cell current; 1 A is characterization amplitude, not a hardware current limit. Tails are retained and reported.', ...
    'scope','Same requested trajectory, existing cell, ideal drive, no new QP or whole-robot/hardware claim; no automatic pack selection or runtime modification.');
names={'run_pack_series_diagnostic','robotenergy.replayDemand','robotenergy.advancePlant', ...
    'robotenergy.instantaneousPack','robotenergy.conservativePackBound','robotenergy.lookup','robotenergy.start'};
meta.analysis_files=cellfun(@which,names,'UniformOutput',false);
meta.analysis_file_hashes=cellfun(@sha256_file,meta.analysis_files,'UniformOutput',false);
writeJson(outputRoot,'manifest',meta); writeJson(outputRoot,'reference_bounds',reference);
try
    assert(peakTau<=bundle.cfg.ENERGY.motorTorqueLimit,'packSeries:MotorTorque','Changing the pack cannot repair motor torque exceedance.');
    candidates=cell(1,numel(options.series_counts));
    for k=1:numel(options.series_counts)
        ns=options.series_counts(k);
        caseReference=struct('n_series',ns,'characterization_current_A',reference.characterization_current_A);
        candidate=struct('n_series',ns,'n_parallel',1,'cell_capacity_Ah',diagnosticBattery.C_nom_Ah, ...
            'required_full_charge_bus_capability_V',ns*reference.ocv_max_V, ...
            'maximum_ideal_no_sag_bus_voltage_V',ns*reference.ocv_max_V,'cases',{{}}, ...
            'all_requested_start_soc_cases_qualified',false);
        if candidate.maximum_ideal_no_sag_bus_voltage_V<peakV
            candidate.status='not_replayed_necessary_voltage_bound';
            candidate.reason='Even maximum cell OCV without load sag cannot supply the retained requested voltage peak.';
            candidate.initial_soc_cases_not_replayed=options.initial_soc;
        else
            candidate.status='replayed_or_verified_baseline';
            for j=1:numel(options.initial_soc)
                initialSoc=options.initial_soc(j);
                if ns==20 && initialSoc==.95 && ~isempty(options.baseline_selection_root)
                    result=reuseBaseline(options.baseline_selection_root,bundle,inputResult,inputManifest);
                else
                    candidateBundle=bundle;
                    candidateBundle.cfg.BATTERY.pack_voltage=ns*diagnosticBattery.assumptions.cell_nominal_voltage_reported_V;
                    result=robotenergy.replayDemand(candidateBundle,initialSoc,caseReference,1,d,outputRoot);
                    result.origin='new_diagnostic_physical_replay';
                    result.source_sha=sourceSha;
                end
                result.partial_trace_semantics=['On failure, final SOC/charge/energy/duration describe the last evaluated physical state, ' ...
                    'possibly partway through a failed MPC interval. Only completed midpoint rows enter observed-tail duration; ' ...
                    'whole-mission P95/RMS and conservation errors remain undefined for an incomplete trace.'];
                candidate.cases{end+1}=result;
                writeJson(outputRoot,sprintf('candidate_%dS1P',ns),candidate);
            end
            candidate.all_requested_start_soc_cases_qualified=all(cellfun(@(c)c.qualified,candidate.cases));
        end
        candidates{k}=candidate;
        writeJson(outputRoot,sprintf('candidate_%dS1P',ns),candidate);
    end
    report=struct('schema',meta.schema,'source_sha',sourceSha,'source_dirty',false,'candidates',{candidates}, ...
        'input_demand_sha256',inputResult.sample_file_sha256,'input_mission_duration_s',N*d.dt, ...
        'input_mission_energy_Wh',energy,'sampled_peak_required_voltage_V',peakV, ...
        'sampled_peak_motor_torque_Nm',peakTau,'motor_torque_limit_Nm',bundle.cfg.ENERGY.motorTorqueLimit, ...
        'fixed_simulation_specification_selected',false,'runtime_configuration_modified',false, ...
        'baseline_pack_unchanged',true,'training_ready',false,'physical_hardware_qualified',false, ...
        'continuous_peak_upper_bound_proven',false, ...
        'limitations','Passing applies only to this recorded requested trajectory and starting SOC. It does not qualify low-SOC operation, all RL actions, a learned policy, or real actuator voltage/speed/thermal limits.');
    assert(strcmp(sourceSha,gitValue(src,'rev-parse HEAD')) && isempty(gitValue(src,'status --porcelain')), ...
        'packSeries:SourceChanged','Source changed during the campaign.');
    writeJson(outputRoot,'result',report);
catch exception
    writeJson(outputRoot,'failure',struct('source_sha',sourceSha,'identifier',exception.identifier, ...
        'message',exception.message,'stack',{exception.stack}));
    rethrow(exception)
end
end

function r=reuseBaseline(root,bundle,inputResult,inputManifest)
manifest=jsondecode(fileread(fullfile(root,'manifest.json')));
selection=jsondecode(fileread(fullfile(root,'selection.json')));
assert(strcmp(manifest.input_sample_file_sha256,inputResult.sample_file_sha256) && ...
    strcmp(manifest.input_configuration_sha256,inputManifest.input_configuration_sha256), ...
    'packSeries:BaselineInput','Baseline used different demand or configuration.');
assert(selection.selected.n_series==20 && selection.selected.n_parallel==1, ...
    'packSeries:BaselinePack','Reusable baseline must be 20S1P.');
cases=selection.selected.cases; index=find([cases.initial_soc]==.95);
assert(isscalar(index),'packSeries:BaselineCase','Expected one 95% baseline case.');
r=cases(index);
assert(r.completed_full_trace && r.qualified,'packSeries:BaselineIncomplete','Baseline must be a complete qualified replay.');
path=fullfile(root,r.file);
assert(strcmp(sha256_file(path),r.file_sha256),'packSeries:BaselineHash','Baseline raw trace hash differs.');
cached=load(path,'b'); b=cached.b;
assert(b.n_series==20 && b.n_parallel==1 && b.C_nom_Ah==bundle.cfg.BATTERY.C_nom_Ah && ...
    b.terminal_margin==bundle.cfg.BATTERY.terminal_margin && ...
    strcmp(sha256_file(b.model_options.parameter_source), ...
        sha256_file(bundle.cfg.BATTERY.model_options.parameter_source)), ...
    'packSeries:BaselineModel','Baseline cell capacity, table, reserve or pack differ.');
% Reuse only if the physical core is byte-identical to the recorded one.
names={'advancePlant.m','instantaneousPack.m','lookup.m'};
for k=1:numel(names)
    index=find(endsWith(string(manifest.analysis_files),['/+robotenergy/' names{k}]));
    assert(isscalar(index) && strcmp(sha256_file(which(['robotenergy.' erase(names{k},'.m')])), ...
        manifest.analysis_file_hashes{index}), 'packSeries:BaselineCore','Physical core changed since baseline.');
end
r.origin='reused_verified_20S1P_95pct_baseline'; r.source_sha=selection.source_sha;
r.source_run_root=root; r.source_manifest_sha256=sha256_file(fullfile(root,'manifest.json'));
r.source_selection_sha256=sha256_file(fullfile(root,'selection.json')); r.file=path;
end
function value=gitValue(root,args)
[status,value]=system(sprintf('git -C "%s" %s',root,args)); assert(status==0); value=strtrim(value);
end
function writeJson(root,name,value)
fid=fopen(fullfile(root,[name '.json']),'w'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
