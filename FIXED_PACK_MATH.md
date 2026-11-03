# Fixed-pack mathematical audit and RL qualification

Current default: the [frozen simulation pack](FROZEN_PACK.md) and [resolved runtime](RESOLVED_ENERGY_METHOD.md). The original 4S6P/interval-mean derivation below is retained for historical interpretation.

This note describes the implemented simulated model, not a calibrated robot. The
parameter sources and unresolved component assumptions in FIXED_PACK_REVIEW.md
still apply. No motor, pack, mass, gait, controller limit or reward weight was
changed by the readiness repair.

## Mechanics and electrical boundary

For each leg, inverse kinematics and its analytic body-frame Jacobian give
`tau = -J' * R' * F`: F is the ground force on the foot. Consequently
`tau' * qdot = -F' * v_relative` under the massless-leg contact-load model.
Tests cover both foot displacement and moving/rotating body virtual work.
The transferred swing-knee torque template remains an explicitly uncalibrated
input; these identities do not supply missing limb inverse dynamics.

Joint-to-motor conversion uses `omega_m = N*qdot`. Including the existing
friction terms, motor torque is `tau_joint/(N*eta)` while driving and
`eta*tau_joint/N` while backdriving. The matched MIT equivalent convention uses
`tau_m = 0.075*i`, `v_m = 0.173*i + 0.10*omega_m`. Its algebraic power identity
is `(3/4)*v_m*i = tau_m*omega_m + (3/4)*0.173*i^2`. Per-motor negative power is
discarded under the existing no-regeneration assumption. This verifies the
internal convention, not an independent winding-resistance measurement.

The source foot dynamics use a sampled target with first-order gain 300/s.
The current drive screen uses signed endpoint joint displacement divided by
the 0.01 s MPC interval. It is an **interval-average-speed screen**, not a bound
on instantaneous voltage. For a stationary-body scalar foot servo with fixed
target, the ratio of initial to interval-mean speed is
`k*dt/(1-exp(-k*dt))`, about 3.157 at these settings. Body motion and nonlinear IK
prevent applying this ratio as a universal correction. The existing exceedances
cannot be waived; passing this screen alone does not establish peak-voltage
feasibility. Detailed signed traces are retained for the next design decision.

## Constant-power battery step

At interval-start SOC z, freeze the characterized parameters for the interval:

`a_j = exp(-dt/(R_j*C_j))`, `b_j = (1-a_j)*R_j`.

`E = OCV(z) - sum(a_j*Vrc_j)`, `R = R0 + sum(b_j)`.

For cell power `w = P/(Ns*Np)`, solve `w = i*(E-R*i)` using the discharge
high-voltage root `i = 2*w/(E+sqrt(E^2-4*R*w))`. This rationalized expression
also gives zero current without cancellation at zero demand. Negative
discriminant or nonpositive E means the requested model interval is unsupplyable.
Book `I_pack = Np*i`, `V_pack = Ns*(E-R*i)`, charge `I_pack*dt` and terminal
energy `V_pack*I_pack*dt/3600`. The charge increment determines the true modeled
SOC decrement `i*dt/(3600*C_nom_Ah)`.

## Exact streaming EKF version 2

The old EKF used a diagonal transition matrix and a tabulated OCV slope despite
SOC-dependent RC parameters and PCHIP OCV. Version
`fixed_pack_exact_jacobian_ekf_v2` linearizes the actual discrete model.
Current is an observed input held fixed for differentiation; the true plant SOC
does not enter the estimator update.

With `tau_j=R_j*C_j`, parameter derivatives from their piecewise-linear lookup,
and prior estimated SOC z:

`a'_j = a_j*dt*(R'_j*C_j + R_j*C'_j)/tau_j^2`.

`F(1,1)=1`, `F(j+1,j+1)=a_j`,
`F(j+1,1)=a'_j*(Vrc_j-R_j*i)+(1-a_j)*R'_j*i`.

Predict end SOC `z_plus=z-i*dt/Q` and RC voltages using prior-SOC parameters.
The measurement reconstructs interval-start SOC `z_plus+i*dt/Q`, so the voltage
model uses the same timing as the plant:

`h(x_plus)=OCV(z_plus+i*dt/Q)-R0(z_plus+i*dt/Q)*i-sum(Vrc_plus)`.

`H=[OCV'(z_plus+i*dt/Q)-R0'(z_plus+i*dt/Q)*i, -1, -1, -1]`.

OCV' is the derivative of the actual PCHIP polynomial. Interior parameter knots
use right derivatives, the last knot uses the left derivative, and clamped
lookup outside the table has zero derivative. The predicted SOC is not clipped
before reconstructing interval-start SOC; the posterior is projected into the
table domain. Joseph covariance updating is retained. Noise remains the
manuscript's **per-update** assumed covariance; no independent accuracy claim
follows from a plant and estimator sharing parameters and noiseless inputs.

Unversioned archived states and configurations without an estimator version keep
the old estimator. The historical batch estimator is unchanged. Its difference
from v2 is reported as a version comparison, not a failed equality check.

## Failure and reporting contracts

- A voltage/torque screen exceedance retains its diagnosed interval, but that
  interval cannot qualify a target crossing as feasible. Geometric crossing time
  remains explicitly separate for diagnosis.
- An IK-domain, pack-power or below-table SOC error has no accepted energy
  interval. Roll back accepted state/time/charge, preserve attempted state/load
  and exception, and return a classified terminal with finite RL outputs.
- Unexpected implementation errors preserve attempt evidence and rethrow.
  Numerical QP success is not relabeled as a solver failure.

## Reproduction and promotion

From a clean checkout without `rlEnv_MPC_R.mat`, use MATLAB R2026a with the
required toolboxes:

```matlab
run_fixed_pack_readiness(fullfile(fileparts(pwd), ...
    'BeyondTorque-Local-Artifacts','my-new-readiness-run'))
```

This runs selected mathematical/regression tests, captured native RL interface
validation, fixed short/full mission cases and the existing low-SOC crosscheck.
Every case retains exact-state evidence, and mission cases export per-joint
signed drive traces. The report distinguishes executable RL transitions from
mission, SOC-range and reward/observation qualification. It never promotes
training automatically. Passing these checks alone does not close the earlier
adverse-domain, observation-sufficiency or baseline-relative learned-value gates.

For a single deterministic policy-interface test from a fresh checkout:

```matlab
run_phase3_policy_evaluation('mission_average', ...
    fullfile(fileparts(pwd),'BeyondTorque-Local-Artifacts','my-policy-probe'), ...
    struct('max_decisions',1))
```

Fresh evaluation uses continuous references, the existing tight one-shot rescue,
and target stopping without creating a legacy worker bundle. Saved worker bundles
retain their historical defaults, and explicit caller modes take precedence.
The captured `Phase3MpcEnvironment` remains the traceable RL interface; this
command evaluates a fixed policy and does not train an actor.
