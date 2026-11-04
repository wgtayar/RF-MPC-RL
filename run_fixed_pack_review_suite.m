function run_fixed_pack_review_suite(root)
% Usage: from a clean checkout alongside RL-MPC-Monitor. New output root only.
assert(~isfolder(root) && ~isfile(root),'Review directory already exists.'); mkdir(root);
src=bootstrap_RF_MPC_RL();
[ok,sha]=system(sprintf('git -C "%s" rev-parse HEAD',src)); assert(ok==0);
[ok,status]=system(sprintf('git -C "%s" status --porcelain',src)); assert(ok==0 && isempty(strtrim(status)));
[ok,branch]=system(sprintf('git -C "%s" branch --show-current',src)); assert(ok==0);
cfg=setup_RL_MPC('legacy_frozen_v1'); cfg=cfg.cfg;
meta=struct('source_sha',strtrim(sha),'branch',strtrim(branch),'dirty',false,'matlab',version, ...
    'timestamp',char(datetime('now','TimeZone','UTC')),'seed',cfg.RNG_SEED,'training',false, ...
    'learning_rates_used',[],'exploration_noise_used',[],'reward',cfg.REWARD.version, ...
    'battery',cfg.BATTERY,'energy',cfg.ENERGY,'governor',[cfg.GAMMA_V_MIN cfg.GAMMA_V_MAX cfg.DGAMMA_V_MAX], ...
    'workbook_sha256',sha256_file(cfg.BATTERY.model_options.parameter_source), ...
    'knee_csv_sha256',sha256_file(cfg.PROXY.kneeCsv));
saveJson(root,'manifest',meta);
tests={'testFixedPackEnergy','testBatteryManuscriptConfig','testPhase3BatteryFeedback','testMpcReplayTimeGrid','testReferencePositionContinuity','testPhase3TargetStop'};
files=cellfun(@(n)fullfile(src,'tests',[n '.m']),tests,'UniformOutput',false);
r=runtests(files); writetable(table(string({r.Name})',[r.Passed]',[r.Failed]',[r.Incomplete]', ...
    'VariableNames',{'name','passed','failed','incomplete'}),fullfile(root,'tests.csv')); assertSuccess(r);
v=run_phase3_environment_validation(fullfile(root,'captured_env'),true);
saveJson(root,'interface',struct('rows',v.qp_steps,'reset_equal',v.reset_equal,'dataset_verified',true,'toolbox',v.toolbox_validation_executed));
% A few metres tests target crossing; 320m/600s is the actual configured mission.
for item={{3,10,'slow'},{320,600,'mission_average'},{320,600,'fast'}}
    x=item{1}; m=run_fixed_pack_mission_review(x{:}); saveJson(root,['mission_' x{3}],m);
end
speed=run_energy_review([.5 .54 .57 .60 .66 .70],8);
writetable(speed,fullfile(root,'speed_probe.csv'));
end
function saveJson(root,name,data)
f=fopen(fullfile(root,[name '.json']),'w'); assert(f>=0); closeFile=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(data,'PrettyPrint',true));
end
