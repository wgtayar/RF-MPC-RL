# Continuous speed transitions and RL control authority

Design record: 2026-11-11. This describes the `velocity_continuous_v2` reference and `bounded_transition_v1` acceleration contract. Qualification results belong in the associated run record; this document makes no full-mission, low-SOC, learned-policy, or hardware-performance claim.

## Scope and unchanged study conditions

- The physical study remains the fixed 9S1P, 2 Ah pack, 15 W auxiliary consumption, 90% driver efficiency, and the existing single-body, massless-leg model.
- Velocity sets the desired forward speed throughout the mission. It was already reapplied after startup; the old defect concerned how later transitions were generated.
- Acceleration is a positive desired-velocity ramp magnitude, used for both speeding up and slowing down. It is not a bound on actual body acceleration, motor torque, or battery current.
- R remains an MPC force-tracking weight. This change does not enlarge R authority or add policy actions.
- The default supervisor retains five-second decisions and a maximum target-speed change of 0.024 m/s per decision. The target-change cap and the within-transition acceleration limit are different quantities.

## Why the historical acceleration action was ineffective later

The historical reference compared absolute simulation time with `v_cmd / a_cmd`. After that startup interval, its desired velocity was immediately `v_cmd`. Changing acceleration then altered only the constant term in desired position; the position-continuity adapter canceled that constant. Consequently acceleration alone could not shape later speed transitions.

The old governor also clamped acceleration to `[max(A_MIN, v_cmd/TACC_MAX), min(A_MAX, v_cmd/TACC_MIN)]`. With the default request and gamma range, all acceleration actions collapsed to 0.5 m/s² at a 1.0 m/s target, and above that target the speed-derived lower bound exceeded the nominal action maximum. This startup-duration rule did not express acceleration authority independent of target speed.

Historical configurations without the new fields retain their old behavior for reproduction. A new reference mode requires its own saved configuration; it must not silently reinterpret an archived policy.

## Acceleration mapping

For clipped acceleration action `gamma_a`, define:

```text
a_low  = max(A_MIN, a_req * GAMMA_A_MIN)
a_high = min(A_MAX, a_req * GAMMA_A_MAX)
fraction = (gamma_a - GAMMA_A_MIN) / (GAMMA_A_MAX - GAMMA_A_MIN)
a_cmd = a_low + fraction * (a_high - a_low)
```

The bounds must be finite, positive and strictly ordered. With the current constants `a_req = 1`, `A_MIN = 0.2`, `A_MAX = 4`, and gamma range `[0.1, 0.5]`, every action maps onto `[0.2, 0.5] m/s²`, independently of target speed. Equivalently, a normalized agent action `u_a` in `[-1,1]` gives `a_cmd = 0.35 + 0.15*u_a`.

This reuses the selected acceleration range while removing its target-speed dependence and clipped dead zone. The numerical bounds are declared controller-design choices, not measured hardware limits. In particular, a 0.5333 m/s startup at 0.2 m/s² now takes about 2.67 seconds; the old two-second startup constraint is intentionally absent from this mode.

Source: [governor](apply_command_governor.m), [normalized action mapping](energy_mission_action_transform.m), [configuration](setup_energy_mission_rl.m).

## Exact desired trajectory

At a command boundary `t0`, evaluate the retained desired trajectory to obtain `x0` and `v0`. For new target speed `v_target` and positive magnitude `a`, define:

```text
delta = v_target - v0
b = sign(delta) * a
T = abs(delta) / a
tau = max(t - t0, 0)
r = min(tau, T)
c = max(tau - T, 0)

x_desired(t) = x0 + v0*r + 0.5*b*r^2 + v_target*c
v_desired(t) = v0 + b*r
a_desired(t) = b for tau < T, otherwise 0
```

The completed ramp explicitly returns `v_target`. When the target is unchanged, `T = 0` and the trajectory simply continues at that speed. Position and velocity are continuous at retargeting and ramp completion. Acceleration is piecewise constant and can jump; this is not a jerk-limited motion planner.

If a new command arrives during an unfinished ramp, the new segment begins from the old segment's desired position and velocity at that instant. It does not restart from zero or reset to measured body velocity. A change in acceleration alone reshapes an unfinished ramp; after reaching an unchanged target, it correctly has no effect until a later speed transition.

The MPC sees these references over its prediction horizon. The foot-placement calculation also receives the generated velocity reference. Actual body response remains determined by MPC forces and the robot dynamics, including their constraints and tracking error.

Source: [update](update_velocity_reference.m), [evaluation](evaluate_velocity_reference.m), [reference generation](fcns/fcn_gen_XdUd.m), [simulation integration](simulate_mpc_horizon.m).

## State, chunking and replay

The saved reference state contains `schema_version`, `mode`, `t_start_s`, `x_start_m`, `v_start_mps`, `v_cmd` and `a_cmd`. A fresh mission initializes desired position and velocity to zero, preserving the mission convention.

An identical repeated command retains the same segment instead of starting another one. Splitting a simulation into chunks therefore does not reset the ramp. Nonzero-time continuation without its reference history is rejected. Evaluation before a segment's start is rejected except for a small floating-point boundary tolerance; past times require the corresponding saved historical segment.

The native state and captured snapshots retain this reference state alongside physical body, force, gait and battery state. Exact disk resume requires the identical archived configuration. Preserve legacy reference adapters and saved policies; a policy using the old acceleration semantics is not automatically qualified for this mode.

At the default five-second cadence, a maximum 0.024 m/s change from an already reached target takes 0.048–0.12 seconds, and the initial 0.5333 m/s startup takes at most about 2.67 seconds. These ramps finish before the next decision. That does not establish that the existing 87-feature observation is Markov sufficient. Shorter decision intervals, different action bounds, or interrupted ramps would require reviewing whether the policy also needs the retained desired velocity and ramp state as explicit observations.

Source: [reference adapter](update_reference_position_offset.m), [captured environment](Phase3MpcEnvironment.m), [state capture](compact_exact_mpc_state.m).

## Connection to current consumption

The coupling is through the modeled motion and forces:

```text
velocity target + ramp magnitude + R weights
  -> MPC reference and ground forces
  -> body/foot kinematics, joint velocities and joint torques
  -> gearbox-adjusted motor torque and motor speed
  -> phase current, copper losses and mechanical power
  -> bus demand, battery current, integrated charge and SOC
```

For each motor, the current model uses `I_phase = torque_motor / phaseKt`, copper loss `0.75*lineResistance*I_phase^2`, and positive electrical demand `max(0, torque_motor*speed_motor + copper_loss)`. Pack-bus demand is the motor-demand sum divided by 0.90, plus 15 W. The battery model supplies the corresponding loaded voltage and current; SOC follows committed charge on the fixed pack. No consumption multiplier is attached to the acceleration or velocity action.

The 100 Hz captured current is charge-equivalent interval mean current; its maximum is not the full resolved within-interval current peak. Faster acceleration need not increase total mission charge: transient effort, travel time, gait response and fixed auxiliary consumption compete. Slower deceleration does not earn regenerative energy because regeneration is absent.

These are reduced-model calculations. Prescribed massless feet, simplified knee swing torque, and absent full impact dynamics limit hardware realism. Source: [drive load](+robotenergy/instantaneousLoad.m), [resolved electrical integration](+robotenergy/sampleResolved.m).

## Meaning and present authority of R

The three policy-controlled weights are x/y/z ground-force tracking weights repeated over all four legs. Their base values are `[0.1, 0.2, 0.1]`. They are neither electrical resistance nor three independent joint-torque costs.

The QP solves force increments `dU` about the current force `Ut`, with Hessian block `R` and linear term `R*(Ut-Ud)`. Its force objective is therefore, up to a constant:

```text
0.5*dU'*R*dU + dU'*R*(Ut-Ud)
  = 0.5*(Ut+dU-Ud)'*R*(Ut+dU-Ud) + constant
```

It penalizes deviation from the nominal desired ground forces `Ud`, rather than solely changes from the previous force. For the current gait, `Ud` splits body weight across stance legs with zero nominal horizontal forces. R affects electrical consumption through the resulting force distribution and tracking, but larger R does not imply less consumed charge.

Each action changes each weight multiplicatively by at most 0.0005 (0.05%), within absolute bounds of ±5% of its base. Starting from the base, the upper bound requires 98 maximum-increase decisions; the lower bound requires 103 maximum-decrease decisions. With the first action at zero and five-second cadence, these are 485 and 510 seconds. A short probe cannot establish useful R adaptation authority.

Source: [bounds](setup_RL_MPC.m), [action application](resolve_supervisory_action.m), [QP objective](fcns_MPC/fcn_get_QP_form_eta.m).

## Qualification and possible additional controls

The transition qualification should restore one identical post-startup state into hold, faster/slower acceleration toward an increased speed, and faster/slower deceleration toward a decreased speed. Archive references, actual motion, forces, current, energy, charge/SOC reconciliation, solver outcomes, and exact replay. Paired acceleration cases must share the target speed; do not assume which consumes less energy. This is bounded action-path evidence, not a replacement for mission and low-SOC qualification.

Before widening R, compare fixed low/base/high weights within its existing ±5% bounds at matched speed, acceleration and SOC. Then check the incremental policy path. Retain speed-only, speed-plus-acceleration, and speed-plus-acceleration-plus-R comparisons so each control's value can be identified.

| Additional candidate | Rationale and prerequisite |
| --- | --- |
| Body-height target | Changes leg geometry and force-to-torque mapping. The clearest next candidate after existing actions; needs continuous ramps, reach/conditioning limits, and stability qualification. |
| Gait period or duty factor | Can change force sharing and swing cost. Requires phase-safe contact updates and consistent swing-torque scaling; current electrical runtime only qualifies the trot template. |
| Swing clearance | Influences foot/joint velocity. Defer until terrain clearance and swing/impact costs are sufficiently represented; the present model could favor unrealistically small clearance. |
| Grouped Q tracking weights | Can trade tracking against effort. Needs independent fall criteria and bounded groups; avoid redundant common scaling of Q and R or weakening stability merely to save modeled energy. |

These are candidate studies, not additional implemented policy actions. Introduce one at a time only after its physical meaning, bounds, observations, failure handling and reproducible comparisons are specified. Pack capacity, current scaling and battery-time compression remain fixed study conditions rather than agent freedoms.
