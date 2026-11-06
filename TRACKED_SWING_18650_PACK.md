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

Cell: the BMS team's Samsung-labelled 18650 ICR "4000 mAh" 3.7 V cell, modeled
with Nancy Zayour and Nagham El Ghossein's pulse characterization (investigator
instruction 2026-11-06; relabelling accepted). Her test applied 0.5C pulses of
6 minutes, each removing 5% SOC, from 100% down to 5%, with OCV reaching 3.03 V:
about 1.9 Ah to 5%, consistent with her stated 2 Ah capacity rather than the
label. The `our params.xlsx` OCV/3RC table and 2 Ah capacity are used exactly
as identified, without rescaling. (Specification revision of 2026-11-05 used
4 Ah with the same table; superseded.)

Specification: `+phase3battery/pack_18650_nancy_2ah_specification.json`,
loaded by `phase3battery.fixedPackConfig(energy_mission_pack_file())`.
Electronics/auxiliary load stays zero and the inverter ideal, by investigator
decision (research control study first).

With 2 Ah, the clean 0.58 m/s mission trace replays to 95% -> 88.13% SOC
(494.80 As); 7S passes from 30/95/100% (1.97/3.76/3.93 V) and stays
voltage-feasible to the 20% floor, while a 25% start reaches the floor
before 320 m. 6S fails from every start.

## Measured plant (clean source f882feb)

Full 320 m native mission, 95% start, requested 0.58 m/s, drive limits
enforced: complete in 554.75 s, 55,475 QP solves (0 failed), 0 voltage or
torque limit intervals, peak required 23.88 V, 3.815 Wh drive energy,
493.48 As, final SOC 91.57%. Interval-level replay of the same trace:
7S passes from 25/95/100% (minimum headroom 1.89/3.83/4.00 V); 6S fails from
every start; 8S passes. Development probes: full-speed request (0.66 m/s)
peaks at 29.88 V (fails on 7S at every start, so the usable speed ceiling falls
with SOC); requested mission pace peaks at 20.98 V. Required voltage rises
with walking speed (22.4 V at 0.545 m/s to 29.9 V at 0.66 m/s).

`energy_mission_reference.json` is this clean mission replayed with the 2 Ah
cell (charge per meter is the reward normalization unit).

## Time/charge preference

`calibrate_energy_mission_weights` on matched 20 m / 50 s native runs (speeds
0.45-0.66 m/s, 95% and 100% starts, clean source f882feb) gives the
indifference weight between slowest and fastest policies
`w* = dQn/(dTn + dQn)` = 0.337 (95%) and 0.334 (100%). Default `time_weight`
is 0.34. Winner by weight: 0.45 m/s below ~0.26, 0.50 m/s to ~0.39, 0.55 m/s
to ~0.51, fastest above. On the full 320 m / 600 s mission the deadline
already excludes requests below about 0.54 m/s, and 7S excludes the fastest
requests, so the learned policy must trade within that SOC-dependent band.
