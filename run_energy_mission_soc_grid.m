function reports = run_energy_mission_soc_grid(checkpointInput,outputRoot,options)
%RUN_ENERGY_MISSION_SOC_GRID Deterministic actor evaluation at fixed start SOCs.
% Declared grid (investigator 2026-11-12): 30, 35, 45, 60, 80, 95, 100%.
% Each start is a separate captured run_energy_mission_policy_evaluation with
% an explicit scenario id. Fixed-speed baselines on the same grid use
% run_energy_mission_comparison with full-mission setup options.
if nargin<3,options=struct();end
d=struct('initial_soc',[.30 .35 .45 .60 .80 .95 1],'max_decisions',inf);
assert(isstruct(options) && all(ismember(fieldnames(options),fieldnames(d))),'energySocGrid:Options','Unknown option.');
for n=fieldnames(d).',if ~isfield(options,n{1}),options.(n{1})=d.(n{1});end,end
assert(~isfolder(outputRoot),'energySocGrid:Exists','Use a new evidence directory.');
mkdir(outputRoot);reports=cell(numel(options.initial_soc),1);rows=cell(0,6);
for k=1:numel(options.initial_soc)
 z=options.initial_soc(k);id=sprintf('soc%03d',round(100*z));
 r=run_energy_mission_policy_evaluation(checkpointInput,fullfile(outputRoot,id), ...
  struct('scenario_id',id,'scenario_overrides',struct('initial_soc',z),'max_decisions',options.max_decisions));
 reports{k}=r;
 rows(end+1,:)={z,string(r.outcome),r.decisions,r.total_reward,r.wall_clock_seconds,id}; %#ok<AGROW>
end
T=cell2table(rows,'VariableNames',{'initial_soc','outcome','decisions','total_reward','wall_s','scenario_id'});
writetable(T,fullfile(outputRoot,'soc_grid_summary.csv'));
end
