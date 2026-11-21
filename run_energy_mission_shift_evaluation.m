function report=run_energy_mission_shift_evaluation(outputRoot,options)
%RUN_ENERGY_MISSION_SHIFT_EVALUATION One shift: back-to-back missions on one battery.
% Protocol A3 (monitor #42): from options.start_soc, the robot walks 320 m
% missions one after another. Each mission restarts the robot from standing;
% the battery (true SOC, RC voltages, estimator mean and covariance) carries
% over. The shift ends normally after a completed mission whose final
% estimated SOC is below options.min_start_soc (30%), or at the first failed
% mission. There is no mission cap (training's cap is a learning device).
%
% Policies:
%   'agent' - deterministic actor of a saved checkpoint (no exploration); the
%             checkpoint's own configuration (including battery_blind) is used.
%   'fixed' - fixed speed options.speed_m_s for every decision (baselines).
%   'b3_shift' - charge-budget rule of protocol A3.3: at each mission start
%             the speed is chosen once from the estimated SOC with
%             energy_mission_b3_shift_speed and the fixed-speed sweep table(s)
%             options.table (path or cell array of paths), then held for the
%             whole mission (R action zero, gamma_a at GAMMA_A_MIN). The table
%             SHA-256 digests are recorded and checked before every mission.
% The policy is built at each mission start from the estimated SOC after the
% reset; missions.csv records the chosen speed (NaN for 'agent'), and
% b3_shift writes each mission's rule audit to mission_<k>_policy_choice.json.
% The shift score G uses options.score_time_weight (A3.0: 0.80) for every
% policy, computed from each mission's totals:
%   completed: 2 - w*tau - (1-w)*kappa;  failed: p - 1 - w*tau - (1-w)*kappa
% with tau, kappa the reward-v3 span-normalized time and charge excesses.
%
% Plant mismatch (protocol A4, evaluation only): plant_resistance_scale
% multiplies the simulated cells' R0..R3, plant_capacity_Ah replaces the
% simulated cell capacity, and plant_soc_estimate_offset sets the EKF's
% initial SOC estimate to true SOC + offset at the first mission (later
% missions carry the estimator). The EKF model, observation, reward scales,
% agent and baseline tables stay nominal (see robotenergy.plantMismatch).
% The plant is recorded in manifest.json, every mission row, result.json,
% the dataset manifest and configuration; defaults are the nominal plant.
%
% Resumable: after every mission the result row and the carried battery are
% saved (state.mat, atomic). Calling again with the same outputRoot and
% options skips completed missions; an interrupted mission folder is renamed
% *_interrupted_<k> and that mission is walked again from the saved battery.
if nargin<2,options=struct();end
options=localOptions(options);
source=bootstrap_RF_MPC_RL();
sha=localGit(source,'rev-parse HEAD');
assert(isempty(localGit(source,'status --porcelain')),'shiftEval:Dirty','Commit source before evaluation.');
canonical=char(java.io.File(char(outputRoot)).getCanonicalPath());
assert(~startsWith(string(canonical),string(source)),'shiftEval:OutputInsideSource', ...
    'Evaluation evidence belongs outside the source checkout.');
[bundle,policyAt,provenance]=localPolicy(options);
[bundle,plant]=localPlant(bundle,options);
c=bundle.cfg;
stateFile=fullfile(outputRoot,'state.mat');
if isfile(stateFile)
    s=load(stateFile);
    assert(isequaln(s.options,options) && strcmp(s.source_sha,sha),'shiftEval:Resume', ...
        'Resume requires identical options and source.');
    missions=s.missions;carried=s.carried;done=s.done;
else
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    missions=struct([]);carried=[];done=false;
    localJson(fullfile(outputRoot,'manifest.json'),struct('schema','energy_mission_shift_evaluation_v1', ...
        'source_sha',sha,'options',options,'policy',provenance,'carried_fields',{{'soc','rc','estimate','cov'}},'plant',plant, ...
        'started',char(datetime('now','TimeZone','local','Format','yyyy-MM-dd''T''HH:mm:ssXXX'))));
end
[~,shiftId]=fileparts(outputRoot);
while ~done
    k=numel(missions)+1;
    assert(k<=options.max_missions,'shiftEval:MissionLimit','Mission safety limit reached.');
    name=sprintf('mission_%02d',k);folder=fullfile(outputRoot,name);
    if isfolder(folder)
        n=1;while isfolder(sprintf('%s_interrupted_%d',folder,n)),n=n+1;end
        movefile(folder,sprintf('%s_interrupted_%d',folder,n));
    end
    metadata=struct('run_id',[shiftId '_' name],'run_type','energy_mission_shift_evaluation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',c.RNG_SEED, ...
        'source_policy',provenance.name,'training',false,'learning_rates_used',[], ...
        'exploration_noise_used',[],'inference_mode','deterministic_no_exploration');
    if ~plant.nominal,metadata.plant_mismatch=bundle.cfg.BATTERY.plant_mismatch;end
    env=EnergyMissionTrainingEnvironment(bundle,folder,metadata,localEnvOptions(c));
    try
        if isempty(carried),observation=reset(env);else,observation=reset(env,carried);end
        [policy,chosenSpeed,choice]=policyAt(env.Native.State.energy.estimate(1));
        e0=env.Native.State.energy;timer=tic;
        while ~env.Native.Done
            [observation,~,~,info]=step(env,policy(observation));
        end
        e=env.Native.State.energy;reason=char(info.terminal_reason);
        metrics=phase3_episode_metrics(env.Native.Writer.Root,env.Native.Config,env.Native.Episode);
        row=localRow(k,e0,e,reason,env.Native.State.t,info.window.distance_end_m,metrics, ...
            energy_mission_scales(env.Native.Config),options.score_time_weight,toc(timer));
        row.chosen_speed_m_s=chosenSpeed;
        row=localPlantRow(row,plant);
        next=struct('soc',e.soc,'rc',e.rc,'estimate',e.estimate,'cov',e.cov);
        root=env.Native.Writer.Root;
        close(env,'shift_evaluation_mission_complete');
        validation=validate_exact_state_dataset(root,true);row.dataset_validated=validation.valid;
        if strcmp(options.policy,'agent')
            actions=validate_energy_mission_policy_actions(root,provenance.actor_for_validation);
            row.actions_reconstructed=actions.valid;
        end
    catch err
        if ~env.Native.Writer.Closed,close(env,'shift_evaluation_exception');end
        rethrow(err)
    end
    if ~isempty(choice),localJson(fullfile(outputRoot,[name '_policy_choice.json']),choice);end
    if isempty(missions),missions=row;else,missions(end+1)=row;end %#ok<AGROW>
    carried=next;
    done=~row.mission_complete || row.end_soc_estimated<options.min_start_soc;
    source_sha=sha; %#ok<NASGU>
    save([stateFile '.tmp'],'missions','carried','done','options','source_sha','-mat');
    movefile([stateFile '.tmp'],stateFile,'f');
    writetable(struct2table(missions,'AsArray',true),fullfile(outputRoot,'missions.csv'));
    fprintf('[shift %s] mission %d (%.2f m/s) %s: %.1f s, %.2f As, SOC %.4f -> %.4f (est %.4f), G_m %.4f\n', ...
        shiftId,k,row.chosen_speed_m_s,row.outcome,row.time_s,row.charge_As,row.start_soc_true,row.end_soc_true,row.end_soc_estimated,row.score);
end
assert(strcmp(sha,localGit(source,'rev-parse HEAD')) && isempty(localGit(source,'status --porcelain')), ...
    'shiftEval:SourceChanged','Source changed during evaluation.');
report=struct('schema','energy_mission_shift_evaluation_v1','source_sha',sha,'options',options, ...
    'policy',rmfield(provenance,'actor_for_validation'),'start_soc',options.start_soc, ...
    'missions_attempted',numel(missions),'missions_completed',sum([missions.mission_complete]), ...
    'end_reason',missions(end).outcome,'walking_time_s',sum([missions.time_s]), ...
    'charge_As',sum([missions.charge_As]),'end_soc_true',missions(end).end_soc_true, ...
    'end_soc_estimated',missions(end).end_soc_estimated,'G',sum([missions.score]), ...
    'score_time_weight',options.score_time_weight,'failed_qps',sum([missions.failed_qps]), ...
    'all_datasets_validated',all([missions.dataset_validated]),'missions',missions);
report.plant=plant;
localJson(fullfile(outputRoot,'result.json'),report);
end

function options=localOptions(options)
d=struct('policy','fixed','checkpoint','','speed_m_s',NaN,'table','','start_soc',.95,'min_start_soc',.30, ...
    'score_time_weight',.80,'max_missions',30,'smoke_distance_m',NaN,'smoke_duration_s',NaN, ...
    'plant_resistance_scale',1,'plant_capacity_Ah',[],'plant_soc_estimate_offset',0);
assert(isstruct(options) && isscalar(options) && all(ismember(fieldnames(options),fieldnames(d))), ...
    'shiftEval:Options','Unknown shift evaluation option.');
for n=fieldnames(d).',if ~isfield(options,n{1}),options.(n{1})=d.(n{1});end,end
options.policy=validatestring(options.policy,{'agent','fixed','b3_shift'});
options.checkpoint=char(options.checkpoint);
if isstring(options.table) || iscell(options.table)
    options.table=cellstr(options.table);options.table=options.table(:).';
    if isscalar(options.table),options.table=options.table{1};end
end
assert(ischar(options.table) || iscellstr(options.table),'shiftEval:Table','The table option must be a path or paths.');
validateattributes(options.start_soc,{'double'},{'scalar','real','>=',.30,'<=',1});
validateattributes(options.min_start_soc,{'double'},{'scalar','real','>',.2,'<',1});
validateattributes(options.score_time_weight,{'double'},{'scalar','real','>=',0,'<=',1});
validateattributes(options.max_missions,{'double'},{'scalar','integer','positive'});
validateattributes(options.plant_resistance_scale,{'double'},{'scalar','real','finite','positive'});
if ~isempty(options.plant_capacity_Ah)
    validateattributes(options.plant_capacity_Ah,{'double'},{'scalar','real','finite','positive'});
end
validateattributes(options.plant_soc_estimate_offset,{'double'},{'scalar','real','finite','>',-1,'<',1});
switch options.policy
    case 'agent'
        assert(~isempty(options.checkpoint),'shiftEval:Checkpoint','An agent evaluation needs a checkpoint.');
    case 'fixed'
        validateattributes(options.speed_m_s,{'double'},{'scalar','real','finite','positive'});
    case 'b3_shift'
        assert(~isempty(options.table),'shiftEval:Table','A b3_shift evaluation needs the sweep table(s).');
        assert(isnan(options.speed_m_s),'shiftEval:Speed','b3_shift chooses its own speed; leave speed_m_s unset.');
end
end

function [bundle,policyAt,provenance]=localPolicy(options)
% policyAt(socEstimate) returns the mission's policy handle, its constant
% speed (NaN for the actor) and an audit struct (empty unless b3_shift).
smoke=struct();
if ~isnan(options.smoke_distance_m)
    smoke=struct('target_distance_m',options.smoke_distance_m,'mission_duration_s',options.smoke_duration_s);
end
if strcmp(options.policy,'agent')
    checkpoint=load_energy_mission_checkpoint(options.checkpoint);
    overrides=smoke;overrides.initial_soc=options.start_soc;
    bundle=energy_mission_evaluation_bundle(checkpoint.bundle,overrides);
    actor=checkpoint.actor;
    policyAt=@(~)deal(@(observation)localActor(actor,observation),NaN,[]);
    provenance=struct('name','deterministic_actor','checkpoint_path',checkpoint.path, ...
        'checkpoint_sha256',checkpoint.sha256,'checkpoint_root',checkpoint.root, ...
        'training_source_sha',checkpoint.training_source_sha, ...
        'episodes_trained',checkpoint.checkpoint_episodes_trained, ...
        'battery_blind',localBlind(bundle.cfg),'actor_for_validation',actor);
else
    setup=struct('initial_soc',options.start_soc,'time_weight',options.score_time_weight);
    for n=fieldnames(smoke).',setup.(n{1})=smoke.(n{1});end
    bundle=setup_energy_mission_rl(setup);c=bundle.cfg;
    if strcmp(options.policy,'fixed')
        normalized=localSpeedAction(c,options.speed_m_s);
        policyAt=@(~)deal(@(~)normalized,options.speed_m_s,[]);
        provenance=struct('name',sprintf('fixed_%.2f',options.speed_m_s),'speed_m_s',options.speed_m_s, ...
            'battery_blind',false,'actor_for_validation',[]);
    else
        paths=cellstr(options.table);
        digests=cellfun(@sha256_file,paths,'UniformOutput',false);
        for v=[1.0 0.9 0.85 0.8 0.75],localSpeedAction(c,v);end   % every candidate is admissible
        policyAt=@(soc)localB3(c,paths,digests,soc);
        provenance=struct('name','b3_shift','rule','energy_mission_b3_shift_speed', ...
            'table_paths',{paths},'table_sha256',{digests},'battery_blind',false,'actor_for_validation',[]);
    end
end
% Evaluation chains missions itself; the training wrapper's chaining is off.
bundle.cfg.ENERGY_RL.options.shift.enabled=false;
if isfield(bundle.cfg,'INITIAL_SOC'),bundle.cfg.INITIAL_SOC.mode='fixed';end
end

function [bundle,plant]=localPlant(bundle,options)
% Declares the evaluation-only plant mismatch on the battery configuration
% (robotenergy.start attaches it to the plant state). Defaults leave the
% configuration untouched: the nominal plant reproduces exactly.
b=bundle.cfg.BATTERY;
assert(~isfield(b,'plant_mismatch'),'shiftEval:Plant','The configuration already declares a plant mismatch.');
capacity=options.plant_capacity_Ah;if isempty(capacity),capacity=b.C_nom_Ah;end
nominal=options.plant_resistance_scale==1 && isempty(options.plant_capacity_Ah) && options.plant_soc_estimate_offset==0;
plant=struct('nominal',nominal,'version','plant_battery_mismatch_v1', ...
    'resistance_scale',options.plant_resistance_scale,'capacity_Ah',capacity, ...
    'nominal_capacity_Ah',b.C_nom_Ah,'soc_estimate_offset',options.plant_soc_estimate_offset);
if nominal,return;end
bundle.cfg.BATTERY.plant_mismatch=struct('version',plant.version, ...
    'resistance_scale',plant.resistance_scale,'capacity_Ah',capacity, ...
    'soc_estimate_offset',plant.soc_estimate_offset,'scope', ...
    ['Protocol A4 evaluation only: the simulated plant cells use scaled R0..R3 and this capacity; ' ...
    'the EKF model, observation, reward scales, agent and baseline tables stay nominal; the EKF ' ...
    'initial SOC estimate is true SOC plus the offset at a fresh (first-mission) start.']);
robotenergy.plantMismatch(bundle.cfg.BATTERY);
end

function row=localPlantRow(row,plant)
row.plant_nominal=plant.nominal;row.plant_resistance_scale=plant.resistance_scale;
row.plant_capacity_Ah=plant.capacity_Ah;row.plant_soc_estimate_offset=plant.soc_estimate_offset;
end

function normalized=localSpeedAction(c,speed)
% Normalized action holding speed: R unchanged (zero), gamma_a at its minimum.
gamma=(speed-c.V_MIN)/(c.V_MAX-c.V_MIN);
assert(gamma>=c.GAMMA_V_MIN-1e-12 && gamma<=c.GAMMA_V_MAX+1e-12,'shiftEval:Speed','Speed outside action bounds.');
physical=[zeros(3,1);min(max(gamma,c.GAMMA_V_MIN),c.GAMMA_V_MAX);c.GAMMA_A_MIN];
[~,audit]=energy_mission_action_transform(zeros(5,1),c);
normalized=(physical-audit.bias)./audit.scale;
end

function [policy,speed,choice]=localB3(c,paths,digests,soc)
assert(isequal(cellfun(@sha256_file,paths,'UniformOutput',false),digests),'shiftEval:TableChanged', ...
    'The sweep table changed during the shift.');
[speed,choice]=energy_mission_b3_shift_speed(paths,soc);
normalized=localSpeedAction(c,speed);
policy=@(~)normalized;
end

function action=localActor(actor,observation)
[value,state]=evaluate(actor,{observation});
assert(isempty(state),'shiftEval:RecurrentActor','Recurrent actors are unsupported.');
action=double(value{1}(:));
assert(numel(action)==5 && all(isfinite(action)) && all(abs(action)<=1+1e-12),'shiftEval:ActorBounds', ...
    'Actor must emit five normalized bounded actions.');
end

function tf=localBlind(cfg)
tf=isfield(cfg.ENERGY_RL.options,'battery_blind') && cfg.ENERGY_RL.options.battery_blind;
end

function o=localEnvOptions(c)
% Saved checkpoints carry their environment options; a fresh setup has none.
o=struct();if isfield(c,'PHASE3') && isfield(c.PHASE3,'options'),o=c.PHASE3.options;end
if isfield(o,'reward_schema_if_present'),o=rmfield(o,'reward_schema_if_present');end
end

function row=localRow(k,e0,e,reason,time,distance,metrics,scales,w,wall)
p=min(max(distance/scales.target_distance_m,0),1);
tau=(time-scales.minimum_time_s*p)/scales.time_span_s;
kappa=(e.chargeAs-scales.charge_floor_As_per_m*scales.target_distance_m*p)/scales.charge_span_As;
complete=strcmp(reason,'mission_complete');
if complete,score=2-w*tau-(1-w)*kappa;else,score=p-1-w*tau-(1-w)*kappa;end
row=struct('mission',k,'start_soc_true',e0.soc,'start_soc_estimated',e0.estimate(1), ...
    'outcome',reason,'mission_complete',complete,'time_s',time,'distance_m',distance,'progress',p, ...
    'charge_As',e.chargeAs,'energy_Wh',e.energyWh,'end_soc_true',e.soc,'end_soc_estimated',e.estimate(1), ...
    'tau',tau,'kappa',kappa,'score',score,'failed_qps',metrics.failed_qps, ...
    'voltage_limit_steps',metrics.voltage_limit_steps,'torque_limit_steps',metrics.torque_limit_steps, ...
    'wall_min',wall/60,'dataset_validated',false,'actions_reconstructed',true,'chosen_speed_m_s',NaN);
end

function out=localGit(root,args)
[s,out]=git_output(root,args);assert(s==0,'shiftEval:Git','%s',out);out=strtrim(out);
end

function localJson(path,value)
f=fopen(path,'w');assert(f>=0,'shiftEval:Write','Cannot write %s.',path);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
