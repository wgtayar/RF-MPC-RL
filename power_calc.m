function [tpc, Pc] = power_calc(t, X, U, params)
    if nargin < 4 || isempty(params)
        try
            params = evalin('caller', 'p');
        catch
            error('power_calc:params', 'Provide params with field BI (3x3).');
        end
    end
    if ~isfield(params,'BI')
        error('power_calc:BI', 'params.BI (3x3 body inertia) is required.');
    end
    BI = params.BI;

    % Unpack
    t = t(:);
    v = X(:,4:6); % CoM velocity in world
    Rv = X(:,7:15); % rotation matrix entries
    Bw = X(:,16:18); % body-frame angular velocity
    N = size(X,1);

    % Rebuild R for each sample and world-frame angular velocity
    R = zeros(3,3,N);
    for k = 1:N
        R(:,:,k) = reshape(Rv(k,:).', 3,3); % column-major
    end

    % Numerical time-derivative of body-frame omega
    % Use robust first-order on nonuniform grid (gradient handles endpoints)
    domega = zeros(N,3);
    for j = 1:3
        domega(:,j) = gradient(Bw(:,j), t);
    end

    % Helpers
    Pc = zeros(N,1);
    for k = 1:N
        % Total GRF in world frame
        uk = reshape(U(k,:), 3,4);          % columns are legs
        Ftot = sum(uk, 2);                  % 3x1

        % Angular velocity in world
        w_world = R(:,:,k) * Bw(k,:).';     % 3x1

        % Body-frame aux for torque reconstruction
        Bwk = Bw(k,:).';                    % 3x1
        Bwh = [  0     -Bwk(3)  Bwk(2);
                 Bwk(3)   0    -Bwk(1);
                -Bwk(2) Bwk(1)   0     ];
        tau_world = R(:,:,k) * ( BI * domega(k,:).' + Bwh * BI * Bwk );  % 3x1

        % Centroidal mechanical power
        Pc(k) = Ftot.' * v(k,:).' + tau_world.' * w_world;
    end

    % Optional light smoothing if user provided a window in seconds
    if isfield(params,'smoothWin') && ~isempty(params.smoothWin) && params.smoothWin > 0
        % Convert seconds to samples with local dt estimate
        dt_med = median(diff(t));
        win = max(1, round(params.smoothWin / max(dt_med, eps)));
        Pc = movmean(Pc, win, 'Endpoints','shrink');
    end

    tpc = t;
end
