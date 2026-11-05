# Tracked swing model and locked 18650 pack (2026-11-05)

Plant revision for the energy-mission RL profile (`setup_energy_mission_rl`).
Historical profiles (`setup_RL_MPC('legacy_frozen_v1')`, `run_MPC_simulation`,
static/transition sweeps) keep the held-target swing model and the 20S1P/2 Ah
diagnostic pack, so archived evidence reproduces unchanged.

## Why the old swing demanded 67.5 V

At each 10 ms MPC step `fcn_FSM` evaluates the swing Bezier plan `p_d(s)` at
the step start and returns it as a constant target. `dynamics_SRB` then drives
the swing foot with a first-order servo held for the whole interval:

`dp/dt = K (p_d(s(t_k)) - p)`, `K = Kp_sw = 300 1/s`.

The foot therefore lags the moving plan and, at every interval start, jumps
toward a new target. For a fixed target the interval-start speed is
`K dt / (1 - exp(-K dt)) = 3.157` times the interval-average speed. These
sawtooth peaks, not foot-placement distance or torque, set the old peak motor
speed (674 rad/s) and the back-EMF-dominated 67.5 V bus requirement
(`v_m = 0.173 i + 0.10 omega_m`, MIT MiniCheetah constants designed for 24 V).

## Tracked swing (`cfg.SWING.model = 'tracked_bezier_v1'`)

`fcn_FSM` now also returns its per-leg swing plan (`swingRef`: active flag,
`Ta`, `Tb`, the 3-by-6 Bezier control points it already built). With the
tracked model, `simulate_mpc_horizon` attaches it to `p.swing_reference`, and
`dynamics_SRB` evaluates the plan continuously inside the interval:

`s(t) = clip((t - Ta)/(Tb - Ta), 0, 1)`,
`dp/dt = p_d'(s) / (Tb - Ta) + K (p_d(s(t)) - p)` for swing legs,

with `p_d'(s) = Bezier(5 diff(co), s)` (zero once `s = 1`). The tracking error
`e = p_d - p` obeys `de/dt = -K e`: a foot on the plan moves exactly at the
plan velocity, and the per-step capture-point update of the touchdown target
is absorbed by the same first-order correction. Stance feet keep the original
servo. The plan itself (step timing, capture-point foothold, 0.1 m Bezier
height control points) is unchanged; only the physically implausible held
target inside the interval is removed. The energy model already derives joint
speeds from the same ODE derivative, so demand stays consistent automatically.

`swing_model_of(cfg)` returns the declared model; configurations without
`cfg.SWING` are historical. The bound gait FSM and problem-override replays
are rejected with the tracked model rather than silently mixing models.
Tests: `tests/testTrackedSwing.m`.

## Pack

Cell: Samsung 18650 ICR, 4000 mAh, 3.7 V nominal, as tested by the BMS team
(investigator instruction 2026-11-05). The electrical table is the existing
team-characterized `our params.xlsx` (OCV 3.03-4.18 V over 5-100% SOC,
consistent with an ICR/NMC-type cell), unchanged and hash-locked. Only the
capacity changes to 4 Ah. Resistances are NOT rescaled for the larger
capacity: if the table came from a smaller cell this overstates sag, which is
conservative for voltage sizing. Replace the table with BMS-team measurements
of the 4 Ah cell when available and qualify a new specification.

Specification: `+phase3battery/pack_18650_4ah_specification.json`, loaded by
`phase3battery.fixedPackConfig(energy_mission_pack_file())`. Series count,
selection rule and evidence are recorded in the specification and in the
monitor run report. Electronics/auxiliary load stays zero and the inverter
ideal, by investigator decision (research control study first).
