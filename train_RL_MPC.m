function report = train_RL_MPC(maxEpisodes)
%TRAIN_RL_MPC Compatibility entry to fresh captured energy-mission training.
% Saved legacy worker environments/checkpoints are intentionally not reused.
    src=bootstrap_RF_MPC_RL();
    if nargin<1,maxEpisodes=150;end
    validateattributes(maxEpisodes,{'double'},{'scalar','integer','positive','finite'});
    stamp=char(datetime('now','TimeZone','UTC','Format','yyyyMMdd''T''HHmmssSSS'));
    outputRoot=fullfile(fileparts(src),'BeyondTorque-Local-Artifacts',['energy-training-' stamp]);
    report=train_energy_mission_rl(outputRoot,struct('mode','train','max_episodes',maxEpisodes));
end
