function report = run_pack_sizing_analysis(inputRoot,outputRoot)
%RUN_PACK_SIZING_ANALYSIS Replay one uncensored requested-load trace into packs.
% This is a provisional interval-mean-speed comparison, not achieved motion,
% a peak-voltage qualification, a hardware rating, or automatic pack selection.
% Input configuration.mat contains b; demand.mat contains drive (N-by-53).
src=bootstrap_RF_MPC_RL();
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'packSizing:Exists','Use a new output directory.');
configurationFile=fullfile(inputRoot,'configuration.mat');
demandFile=fullfile(inputRoot,'demand.mat');
loaded=load(configurationFile,'b'); b=loaded.b;
loaded=load(demandFile,'drive'); drive=loaded.drive;
assert(isnumeric(drive) && isreal(drive) && size(drive,2)==53 && ...
    size(drive,1)>0 && all(isfinite(drive),'all'),'packSizing:Trace','Require a finite N-by-53 drive trace.');
dt=drive(:,2)-drive(:,1);
assert(all(dt>0) && all(drive(:,3:5)>=0,'all'),'packSizing:Trace','Intervals and requested loads must be valid.');
tol=64*eps(max(1,max(abs(drive(:,1:2)),[],'all')));
assert(all(abs(drive(2:end,1)-drive(1:end-1,2))<=tol), ...
    'packSizing:Continuity','The sizing trace must have contiguous intervals.');
assert(abs(drive(1,1))<=tol,'packSizing:Origin','Sizing replay starts with fresh cells at time zero.');
cfg=b.cfg;
assert(~cfg.BATTERY.use_pack_sizing,'packSizing:Fixed','Input must use an explicit fixed pack.');
assert(cfg.BATTERY.SOC_init==.95 && cfg.BATTERY.terminal_margin==.2, ...
    'packSizing:Scope','This frozen comparison requires original95% start and20% mission floor.');
supportA=cfg.BATTERY.assumptions.characterization_discharge_C*cfg.BATTERY.C_nom_Ah;
assert(supportA==1,'packSizing:Cell','The retained manuscript profile has1A characterization amplitude.');
initialSocs=[.95 .25]; seriesCounts=4:6; parallelCounts=1:6;
mkdir(outputRoot);
sourceSha=gitValue(src,'rev-parse HEAD'); sourceStatus=gitValue(src,'status --porcelain');
meta=struct('schema','pack_sizing_requested_mean_trace_v1','source_sha',sourceSha, ...
    'source_branch',gitValue(src,'branch --show-current'),'source_dirty',~isempty(sourceStatus), ...
    'source_status',sourceStatus,'matlab',version,'seed',cfg.RNG_SEED, ...
    'reward_version',cfg.REWARD.version,'training',false,'episodes_trained',0, ...
    'learning_rates_used',[],'exploration_noise_used',[], ...
    'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'input_root',char(inputRoot),'configuration_sha256',sha256_file(configurationFile), ...
    'demand_sha256',sha256_file(demandFile),'series_counts',seriesCounts, ...
    'parallel_counts',parallelCounts,'initial_soc_scenarios',initialSocs, ...
    'mission_soc_floor',cfg.BATTERY.terminal_margin, ...
    'characterization_current_A',supportA, ...
    'current_threshold_meaning','0.5C laboratory characterization amplitude; not a continuous or pulse current rating', ...
    'quantile','time-weighted empirical inverse CDF, including startup; no interpolation', ...
    'interpretation','Requested trajectory, interval-average joint speed; all candidate qualification is provisional and does not establish instantaneous drive feasibility.');
inputManifest=fullfile(inputRoot,'manifest.json');
if isfile(inputManifest)
    meta.input_manifest=jsondecode(fileread(inputManifest));
    meta.input_manifest_sha256=sha256_file(inputManifest);
end
files={'run_pack_sizing_analysis.m','+robotenergy/weightedQuantile.m', ...
    '+robotenergy/advance.m','+robotenergy/start.m','+robotenergy/lookup.m', ...
    '+robotenergy/linearizeEstimator.m','+robotenergy/estimatorVersion.m'};
meta.analysis_source_files=files;
meta.analysis_source_hashes=cellfun(@(f)sha256_file(fullfile(src,f)),files,'UniformOutput',false);
writeJson(outputRoot,'manifest',meta);
try
    torque=abs(drive(:,6:17)); speed=abs(drive(:,18:29));
    phase=abs(drive(:,30:41)); voltage=abs(drive(:,42:53));
    axes=repmat((1:3)',4,1); legs=repelem((1:4)',3);
    perJoint=table(legs,axes,'VariableNames',{'leg','axis'});
    for item={{'joint_torque_Nm',torque},{'joint_speed_rad_s',speed}, ...
            {'phase_current_A',phase},{'motor_voltage_V',voltage}}
        name=item{1}{1}; data=item{1}{2};
        perJoint.([name '_p95'])=arrayfun(@(j)robotenergy.weightedQuantile(data(:,j),dt,.95),1:12)';
        perJoint.([name '_maximum'])=max(data,[],1)';
    end
    writetable(perJoint,fullfile(outputRoot,'per_joint_statistics.csv'));
    demand=struct('samples',size(drive,1),'duration_s',sum(dt), ...
        'requested_energy_Wh',sum(drive(:,3).*dt)/3600, ...
        'power_p95_W',robotenergy.weightedQuantile(drive(:,3),dt,.95), ...
        'power_max_W',max(drive(:,3)), ...
        'max_joint_torque_p95_Nm',robotenergy.weightedQuantile(max(torque,[],2),dt,.95), ...
        'max_joint_torque_peak_Nm',max(torque,[],'all'), ...
        'max_motor_phase_current_p95_A',robotenergy.weightedQuantile(max(phase,[],2),dt,.95), ...
        'max_motor_phase_current_peak_A',max(phase,[],'all'), ...
        'required_voltage_p95_V',robotenergy.weightedQuantile(drive(:,4),dt,.95), ...
        'required_voltage_peak_V',max(drive(:,4)), ...
        'motor_torque_p95_Nm',robotenergy.weightedQuantile(drive(:,5),dt,.95), ...
        'motor_torque_peak_Nm',max(drive(:,5)), ...
        'motor_torque_limit_Nm',cfg.ENERGY.motorTorqueLimit, ...
        'motor_torque_exceedance_time_s',sum(dt(drive(:,5)>cfg.ENERGY.motorTorqueLimit)), ...
        'current_scope','Motor phase-current statistics are not battery or pack-current statistics.');
    writeJson(outputRoot,'demand_statistics',demand);
    cases=cell(numel(initialSocs)*numel(seriesCounts)*numel(parallelCounts),1); n=0;
    for initialSoc=initialSocs
        for ns=seriesCounts
            for np=parallelCounts
                n=n+1;
                cases{n}=replayCandidate(cfg,drive,dt,initialSoc,ns,np,supportA,outputRoot);
                fprintf('Pack sizing %d/%d: SOC%.0f %dS%dP %s, I95=%gA, headroom=%gV\n', ...
                    n,numel(cases),100*initialSoc,ns,np,cases{n}.status, ...
                    cases{n}.cell_current_p95_A,cases{n}.minimum_mean_voltage_headroom_V);
            end
        end
    end
    candidates=struct2table(vertcat(cases{:}));
    writetable(candidates,fullfile(outputRoot,'candidates.csv'));
    summary=struct('initial_soc',{},'p95_supported_min_cell_designs',{},'peak_supported_min_cell_designs',{});
    for k=1:numel(initialSocs)
        subset=candidates(candidates.initial_soc==initialSocs(k),:);
        summary(k).initial_soc=initialSocs(k);
        summary(k).p95_supported_min_cell_designs=minimumDesigns(subset,'p95_current_supported');
        summary(k).peak_supported_min_cell_designs=minimumDesigns(subset,'peak_current_supported');
    end
    both=candidates(candidates.initial_soc==initialSocs(1),:);
    for k=1:height(both)
        matched=candidates.n_series==both.n_series(k) & candidates.n_parallel==both.n_parallel(k);
        both.mean_model_qualified(k)=all(candidates.mean_model_qualified(matched));
        both.p95_current_supported(k)=all(candidates.p95_current_supported(matched));
        both.peak_current_supported(k)=all(candidates.peak_current_supported(matched));
    end
    report=struct('schema',meta.schema,'source_sha',sourceSha,'source_dirty',meta.source_dirty, ...
        'demand',demand,'candidate_count',height(candidates),'candidates',{cases}, ...
        'minimum_cell_mean_model_comparisons',summary, ...
        'both_scenarios_p95_supported_min_cell_designs',minimumDesigns(both,'p95_current_supported'), ...
        'both_scenarios_peak_supported_min_cell_designs',minimumDesigns(both,'peak_current_supported'), ...
        'instantaneous_voltage_qualified',false,'pack_frozen',false,'training_ready',false, ...
        'decision','No final pack selection: this helper only screens interval-mean speed. A separate instantaneous-voltage audit must pass before a fixed pack can be declared feasible. These comparisons are provisional.', ...
        'search_scope','4/5/6 series and1..6 parallel are a finite simulation comparison, not hardware voltage/current ratings;95% and25% starts are sizing scenarios, not an approved RL reset distribution.');
    assert(strcmp(sourceSha,gitValue(src,'rev-parse HEAD')) && ...
        strcmp(sourceStatus,gitValue(src,'status --porcelain')), ...
        'packSizing:SourceChanged','Source Git state changed during analysis.');
    writeJson(outputRoot,'analysis',report);
catch exception
    writeJson(outputRoot,'failure',struct('source_sha',sourceSha, ...
        'identifier',exception.identifier,'message',exception.message,'stack',{exception.stack}));
    rethrow(exception)
end
end

function r=replayCandidate(cfg,drive,dt,initialSoc,ns,np,supportA,outputRoot)
b=cfg.BATTERY; b.n_series=ns; b.n_parallel=np; b.SOC_init=initialSoc; b.metric_init=100*initialSoc;
s=robotenergy.start(b); floorSoc=b.terminal_margin;
% The upper bound permits no voltage sag, no RC polarization, and no SOC
% depletion. Therefore violating it proves failure without replay, independent
% of the chosen parallel count. PCHIP extrema are included, not guessed.
maxOcv=maximumOcv(s,s.grid(1),initialSoc);
voltageUpper=ns*maxOcv; energyWh=sum(drive(:,3).*dt)/3600;
minimumCellChargeAh=energyWh/(ns*np*maxOcv);
availableCellChargeAh=b.C_nom_Ah*(initialSoc-floorSoc);
r=struct('initial_soc',initialSoc,'n_series',ns,'n_parallel',np,'cell_count',ns*np, ...
    'nominal_capacity_Ah',np*b.C_nom_Ah, ...
    'status','not_started','replayed',false,'completed_demand',false,'accepted_samples',0, ...
    'accepted_duration_s',0,'statistics_scope','not_replayed', ...
    'cell_current_p95_A',NaN,'cell_current_rms_A',NaN,'cell_current_max_A',NaN, ...
    'pack_current_p95_A',NaN,'pack_current_max_A',NaN, ...
    'time_above_characterization_A_s',NaN,'longest_above_characterization_A_s',NaN, ...
    'charge_As',0,'energy_Wh',0,'minimum_soc',initialSoc, ...
    'minimum_mean_voltage_headroom_V',NaN,'voltage_exceedance_time_s',NaN, ...
    'torque_exceedance_time_s',sum(dt(drive(:,5)>cfg.ENERGY.motorTorqueLimit)), ...
    'necessary_voltage_margin_upper_V',voltageUpper-max(drive(:,4)), ...
    'necessary_cell_charge_lower_Ah',minimumCellChargeAh, ...
    'available_cell_charge_Ah',availableCellChargeAh, ...
    'failure_identifier','','failure_sample',NaN,'mean_model_qualified',false, ...
    'p95_current_supported',false,'peak_current_supported',false,'trace_file','','trace_sha256','');
if r.necessary_voltage_margin_upper_V<0
    r.status='not_replayed_necessary_voltage_bound'; return
end
if minimumCellChargeAh>availableCellChargeAh
    r.status='not_replayed_necessary_capacity_bound'; return
end
r.replayed=true; trace=zeros(size(drive,1),9); accepted=0; r.status='complete'; failure=struct();
for k=1:size(drive,1)
    try
        [s,y]=robotenergy.advance(s,drive(k,3),dt(k));
    catch exception
        if ~ismember(exception.identifier,{'robotenergy:PowerLimit','robotenergy:Empty'}), rethrow(exception); end
        r.status=robotenergy.failureReason(exception.identifier);
        r.failure_identifier=exception.identifier; r.failure_sample=k;
        failure=struct('identifier',exception.identifier,'message',exception.message, ...
            'sample',k,'requested_watts',drive(k,3),'dt',dt(k));
        break
    end
    accepted=accepted+1;
    trace(accepted,:)=[drive(k,2),y.cellA,y.packA,y.packV,s.soc,y.socPct,s.chargeAs,s.energyWh,y.packV-drive(k,4)];
    if s.soc<floorSoc
        r.status='mission_soc_floor'; r.failure_sample=k; break
    end
end
trace=trace(1:accepted,:); r.accepted_samples=accepted;
r.completed_demand=accepted==size(drive,1) && strcmp(r.status,'complete');
r.accepted_duration_s=sum(dt(1:accepted)); r.statistics_scope='accepted_prefix';
if r.completed_demand, r.statistics_scope='complete_requested_trace'; end
r.charge_As=s.chargeAs; r.energy_Wh=s.energyWh;
if accepted>0
    w=dt(1:accepted); current=trace(:,2); above=current>supportA;
    r.cell_current_p95_A=robotenergy.weightedQuantile(current,w,.95);
    r.cell_current_rms_A=sqrt(sum(current.^2.*w)/sum(w)); r.cell_current_max_A=max(current);
    r.pack_current_p95_A=robotenergy.weightedQuantile(trace(:,3),w,.95); r.pack_current_max_A=max(trace(:,3));
    r.time_above_characterization_A_s=sum(w(above));
    longest=0; currentRun=0;
    for k=1:numel(w)
        if above(k), currentRun=currentRun+w(k); else, currentRun=0; end
        longest=max(longest,currentRun);
    end
    r.longest_above_characterization_A_s=longest;
    r.minimum_soc=min([initialSoc;trace(:,5)]);
    r.minimum_mean_voltage_headroom_V=min(trace(:,9));
    r.voltage_exceedance_time_s=sum(w(trace(:,9)<0));
    r.mean_model_qualified=r.completed_demand && r.minimum_soc>=floorSoc && ...
        r.voltage_exceedance_time_s==0 && r.torque_exceedance_time_s==0;
    r.p95_current_supported=r.mean_model_qualified && r.cell_current_p95_A<=supportA;
    r.peak_current_supported=r.mean_model_qualified && r.cell_current_max_A<=supportA;
end
r.trace_file=sprintf('replay_soc%02d_%ds%dp.mat',round(100*initialSoc),ns,np);
columns={'time_after_s','cell_current_A','pack_current_A','pack_voltage_V','true_soc_fraction', ...
    'estimated_soc_percent','cumulative_charge_As','cumulative_energy_Wh','mean_voltage_headroom_V'};
save(fullfile(outputRoot,r.trace_file),'trace','columns','s','b','failure','-v7');
r.trace_sha256=sha256_file(fullfile(outputRoot,r.trace_file));
end

function value=maximumOcv(s,lower,upper)
[breaks,coefficients,~,order]=unmkpp(s.ocv);
points=[lower;upper;s.grid(s.grid>=lower & s.grid<=upper)];
for k=1:size(coefficients,1)
    derivative=coefficients(k,1:end-1).*(order-1:-1:1);
    localRoots=roots(derivative);
    localRoots=real(localRoots(abs(imag(localRoots))<eps));
    localRoots=localRoots(localRoots>=0 & localRoots<=breaks(k+1)-breaks(k))+breaks(k);
    points=[points;localRoots(localRoots>=lower & localRoots<=upper)]; %#ok<AGROW>
end
value=max(ppval(s.ocv,points));
assert(isfinite(value) && value>0,'packSizing:OCV','Require a positive finite OCV bound.');
end
function designs=minimumDesigns(candidates,criterion)
eligible=candidates(candidates.(criterion),:);
if isempty(eligible), designs=struct([]); return; end
eligible=eligible(eligible.cell_count==min(eligible.cell_count),{'n_series','n_parallel','cell_count'});
designs=table2struct(eligible);
end
function value=gitValue(root,args)
[status,value]=system(sprintf('git -C "%s" %s',root,args));
assert(status==0,'packSizing:Git','Git provenance failed.'); value=strtrim(value);
end
function writeJson(root,name,value)
file=fopen(fullfile(root,[name '.json']),'w'); assert(file>=0);
cleanup=onCleanup(@()fclose(file)); fprintf(file,'%s\n',jsonencode(value,'PrettyPrint',true));
end
