function rootDir = bootstrap_RF_MPC_RL()
%bootstrap_RF_MPC_RL Add the project folders required by public entry points.

    rootDir = fileparts(mfilename('fullpath'));
    requiredFolders = [
        string(rootDir)
        string(fullfile(rootDir, 'fcns'))
        string(fullfile(rootDir, 'fcns_MPC'))
        string(fullfile(rootDir, 'RL Midtraining Logs'))
    ];

    for folder = requiredFolders.'
        if isfolder(folder)
            addpath(folder);
        end
    end
end
