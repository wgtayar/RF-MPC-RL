function train_RL_MPC()
    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    S = load(cfgPath, 'env', 'agent', 'cfg');
    env = S.env;
    agent = S.agent;
    cfg = S.cfg;

    cfg.LOG.enable = true;
    save(cfgPath, 'cfg', '-append');

    clear rlResetFunction rlStepFunction

    fprintf('[TRAIN START] episodes=%d, decisions/episode=%d, chunks/decision=%d, chunks/episode=%d\n', ...
        600, cfg.EP_STEPS, cfg.APPLY_EVERY, cfg.EP_STEPS * cfg.APPLY_EVERY);

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', 600, ...
        'MaxStepsPerEpisode', cfg.EP_STEPS, ...
        'ScoreAveragingWindowLength', 20, ...
        'Verbose', true, ...
        'Plots', 'training-progress');

    stats = train(agent, env, trainOpts);

    cfg.LOG.enable = false;
    save(cfgPath, 'cfg', '-append');

    save(fullfile(rootDir, 'final_RL_agent.mat'), 'agent', 'stats');
    disp('Training complete.')
end