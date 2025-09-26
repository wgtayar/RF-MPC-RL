function t = get_snapshot_time_or_zero()
    if exist('SimSnapshot_RL.mat','file')
        S = load('SimSnapshot_RL.mat','Sim');
        if isfield(S,'Sim') && isfield(S.Sim,'t') && isfinite(S.Sim.t)
            t = S.Sim.t;
            return;
        end
    end
    t = 0;
end
