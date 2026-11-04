function report=validate_energy_mission_policy_actions(root,actor)
%VALIDATE_ENERGY_MISSION_POLICY_ACTIONS Reconstruct actor and both action maps.
s=load(fullfile(root,'manifest','configuration.mat'),'cfg');cfg=s.cfg;
index=readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
files=dir(fullfile(root,'decisions','*.mat'));
for k=1:numel(files)
 s=load(fullfile(files(k).folder,files(k).name),'event');event=s.event;record=event.record;
 path=fullfile(root,'normalized_actions',sprintf('episode_%06d_decision_%06d.mat',event.context.episode,event.context.decision));
 s=load(path,'audit');audit=s.audit;
 [value,next]=evaluate(actor,{record.observation});
 assert(isempty(next) && isequal(double(value{1}(:)),audit.requested_normalized), ...
  'energyPolicy:ActorReconstruction','Captured normalized action differs from deterministic actor inference.');
 [physical,expected]=energy_mission_action_transform(audit.requested_normalized,cfg);
 assert(isequal(physical,audit.physical_candidate) && isequal(expected.bounded_normalized,audit.bounded_normalized) && ...
  isequal(physical,record.action_execution.candidate_action), ...
  'energyPolicy:ActionReconstruction','Normalized-to-physical action mapping differs from native capture.');
 segment=find(index.first_row<=record.first_mpc_row & index.last_row>=record.first_mpc_row);
 assert(isscalar(segment),'energyPolicy:ActionReconstruction','Missing first-row action snapshot.');
 rows=read_exact_state_rows(fullfile(root,index.file(segment)));row=rows{record.first_mpc_row-index.first_row(segment)+1};
 s=load(fullfile(root,row.snapshot_reference.file),'snapshot');
 [~,execution]=resolve_replay_action(physical,s.snapshot.state,cfg,cfg.PHASE3.lower_R,cfg.PHASE3.upper_R);
 assert(isequaln(execution,record.action_execution),'energyPolicy:GovernorReconstruction','Saved governed action differs from exact replay.');
end
report=struct('valid',true,'actions_reconstructed',numel(files), ...
 'actor_outputs_verified',true,'normalized_mapping_verified',true,'governor_mapping_verified',true);
end
