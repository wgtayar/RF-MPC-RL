function dXdt = dynamics_SRB(t,Xt,Ut,Xd,U_ext,p)

%% parameters
mass = p.mass;
J = p.J;       % inertia tensor in body frame {B}
g = 9.81;

%% decompose
% X = [pc dpc vR wb pf]'
pc = reshape(Xt(1:3),[3,1]);
dpc = reshape(Xt(4:6),[3,1]);
R = reshape(Xt(7:15),[3,3]);
wb = reshape(Xt(16:18),[3,1]);
pf34 = reshape(Xt(19:30),[3,4]);
pfd34 = reshape(Xd(19:30),[3,4]);

% r
r34 = pf34 - repmat(pc,[1,4]);

% GRF
f34 = reshape(Ut,[3,4]);

%% dynamics
ddpc = 1/mass * (sum(f34,2) + U_ext) + [0;0;-g];
dR = R * hatMap(wb);

tau_s = zeros(3,1);       % body torque expressed in {S}
for ii = 1:4
    tau_s = tau_s + hatMap(r34(:,ii)) * f34(:,ii);
end
tau_ext = hatMap(R * p.p_ext) * U_ext;
tau_tot = sum(tau_s,2) + tau_ext;
dwb = J \ (R' * tau_tot - hatMap(wb) * J * wb);

dpf = p.Kp_sw * (pfd34(:) - pf34(:));
% Swing model 'tracked_bezier_v1': swing feet follow the continuous Bezier
% plan (position and velocity feed-forward) instead of chasing a target held
% for the whole interval. Stance feet keep the historical servo above.
if isfield(p,'swing_reference') && ~isempty(p.swing_reference)
    ref = p.swing_reference;
    for leg = 1:4
        if ref.active(leg)
            span = ref.Tb(leg) - ref.Ta(leg);
            s = min(max((t - ref.Ta(leg))/span,0),1);
            co = ref.coefficients(:,:,leg);
            pos = polyval_bz(co,s);
            vel = zeros(3,1);
            if s < 1
                vel = polyval_bz(5*diff(co,1,2),s)/span;
            end
            idx = 3*(leg-1) + (1:3);
            dpf(idx) = vel + p.Kp_sw*(pos - pf34(:,leg));
        end
    end
end

dXdt = [dpc;ddpc;dR(:);dwb;dpf];

end







