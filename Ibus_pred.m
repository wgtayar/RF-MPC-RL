function stats = Ibus_pred(t_pc, Pc, Vbus, opts)
    if nargin < 3 || isempty(Vbus), Vbus = 20; end
    if nargin < 4, opts = struct(); end
    if ~isfield(opts,'I0'),        opts.I0 = 0.5; end
    if ~isfield(opts,'eta'),       opts.eta = 1.0; end
    if ~isfield(opts,'clipNeg'),   opts.clipNeg = true; end
    if ~isfield(opts,'smoothWin'), opts.smoothWin = []; end

    t_pc = t_pc(:); Pc = Pc(:);
    Ip = opts.I0 + opts.eta * (Pc ./ Vbus);

    % if opts.clipNeg
    %     Ip = max(Ip, 0);
    % end

    % Optional smoothing in time
    if ~isempty(opts.smoothWin) && opts.smoothWin > 0
        dt_med = median(diff(t_pc));
        win = max(1, round(opts.smoothWin / max(dt_med, eps)));
        Ip = movmean(Ip, win, 'Endpoints','shrink');
    end

    stats = struct();
    stats.t = t_pc;
    stats.Pc = Pc;
    stats.Ipred = Ip;
    stats.I0 = opts.I0;
    stats.eta = opts.eta;
    stats.Vbus = Vbus;
end
