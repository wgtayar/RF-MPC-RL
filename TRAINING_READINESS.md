# Training readiness contract

Review date: 2026-11-11. This document separates implemented behavior, bounded evidence, and work still required. It is not a claim that the full operating envelope or a trained controller has been validated.

## Update 2026-11-12: gait forecast, speed-change cap, envelope

This supersedes the 0.024 m/s figures below; the older text is kept as the record of that configuration.

- **Gait forecast fixed** (`fcns/fcn_FSM.m`). The MPC contact forecast now keeps alternating stance and swing beyond one gait cycle, and each leg keeps its own swing anchor. The forecast is bit-identical to the previous one wherever the gait cycle (`min(0.3, 0.2/|v|) + 0.15 s`) is at least the 0.4 s horizon, i.e. at 0.80 m/s and below. Earlier evidence at those speeds is unaffected. Above 0.80 m/s it changes: in a 1.0 m/s run, 109 of 500 solves had used a wrong forecast.
- **Per-decision speed-command change cap: 0.2 m/s** (`DGAMMA_V_MAX = 0.2/(V_MAX - V_MIN)`, investigator decision). Any smaller change remains allowed. From 0.5333 m/s, 0.8/1.0/1.2 m/s become reachable after 2/3/4 decisions (first request at 5/10/15 s). At 0.2–0.5 m/s² a 0.2 m/s ramp lasts 0.4–1.0 s and still ends before the next 5 s decision.
- **Envelope kept at 1.2 m/s** (investigator decision). Above the sustainable speed for the current SOC, episodes end with `actuator_limit` (drive voltage) before any QP failure. In all runs to date the drive-limit stop preceded QP infeasibility. The agent is expected to learn that boundary.
- **Massless legs retained** as in RF-MPC; see the monitor's `docs/project/MASSLESS_LEGS_DEFENSE.md`. No gait-timing or clearance actions are added.

Evidence: `RL-MPC-Monitor/runs/gait-envelope-20261112-a/RUN_REPORT.md`.

## Fixed study conditions

- Keep the selected 9S1P, 2 Ah pack fixed across runs and episodes. Do not resize the pack, scale current, or accelerate SOC depletion to produce a learning signal.
- SOC evolves from committed modeled pack current: `SOC(t) = SOC(0) - Q_As(t)/(3600 * 2 Ah)`. Existing declared losses remain 15 W electronics and 90% motor-driver efficiency.
- The present plant is a single rigid body with massless legs, prescribed foot motion, simplified swing torque, and no full contact-impact dynamics. Conclusions are about this reduced model until further validation supports broader claims.
- Operational evaluation includes starts at 95% and 100% SOC followed by natural depletion. Training may use a declared, reproducible initial-SOC curriculum. Evaluation must also compare different starting SOCs. These are separate evaluation conditions, not substitutes for continuous depletion.
- Preserve old source, saved configurations, checkpoints, and evidence. New configuration choices must be recorded with each new run.

## Gap 1: decision timing

The default changes from 50-second to 5-second decisions. Five seconds is the existing MPC chunk duration and a cadence already supported by the comparison interface. The per-decision limits remain unchanged: a speed-command change of at most 0.024 m/s and an R-weight increment magnitude of at most 0.0005 before absolute bounds.

This choice preserves the maximum individual command change and increases the number of decisions in a 600-second episode from 12 to 120. It increases possible adaptation per elapsed second tenfold. It is an explicit algorithmic choice, not a physical acceleration limit or a validated safety threshold.

For a sustained speed request, starting from `v0 = 320/600 = 0.533333... m/s`, the largest permitted upward command after `k` decisions is:

`v_k = min(v_request, v0 + k * 0.024 m/s)`.

The first action is applied at t = 0, so the earliest request time is `(k - 1) * decision_interval_s`.

| Requested speed | Minimum decisions | Earliest request at 5 s | Earliest request at 50 s |
| --- | ---: | ---: | ---: |
| 0.8 m/s | 12 | 55 s | 550 s |
| 1.0 m/s | 20 | 95 s | 950 s |
| 1.2 m/s | 28 | 135 s | 1,350 s |

These are command-accessibility calculations, not achieved body speeds or demonstrated feasible trajectories. The current nominal plant evidence already includes an electrical failure at 1.2 m/s.

The limits deliberately remain per decision in this first change. Changing the cadence again changes the possible adaptation per elapsed second and requires a new declaration and qualification. In particular, a 1-second configuration with the same 0.024 m/s increment has five times the command authority per elapsed second of this 5-second configuration.

### Evidence required for the timing change

1. Configuration/action checks establish the 5-second default, 120-decision horizon, unchanged pack and per-decision caps, and the reachability arithmetic above.
2. Historical explicit configurations retain their old action mapping and sample time; loading a saved agent must continue using its saved configuration.
3. A captured three-decision native probe exercises repeated action application, state continuation, electrical accounting, finite observations/rewards, and the existing terminal handling. The declared up/up/down probe commands approximately 0.557333, 0.581333, and 0.557333 m/s at t = 0, 5, and 10 s.
4. Archive the actual test report and probe result with source revision, full configuration, seed, and artifact hashes. Record failures and interruptions as such.

A successful three-decision probe supports the changed interface over that short trajectory only. It does not qualify the full speed range, long episodes, low-SOC transitions, a learning campaign, or trained-policy performance.

## Remaining gates before a meaningful pilot training run

### A. Transition acceleration (implemented; bounded qualification required)

Fresh energy-mission configurations select `velocity_continuous_v2`: every new speed command starts from the preceding desired position and velocity, ramps with the selected positive acceleration magnitude (negative sign when slowing down), and then cruises. Repeated commands retain the exact saved reference segment, so ordinary chunk boundaries do not restart the ramp. The reference is saved with the physical state for replay. Historical configurations keep their historical mapping and reference mode.

The new `bounded_transition_v1` governor maps the acceleration coordinate across the existing 0.2–0.5 m/s² range independently of target speed. The old startup-time clamp collapsed acceleration authority at speeds of 1.0 m/s and above. This is a desired-reference acceleration, not a physical acceleration or current limit. The action semantics are versioned `action_v3_transition_acceleration`; there remain five action coordinates and 87 observation features.

At the default maximum speed increment of 0.024 m/s, these ramps last 0.048–0.12 s. Both finish before the next five-second decision; acceleration-only changes during cruise correctly have no motion effect. Shorter decision periods that permit mid-ramp decisions require an observation-state sufficiency review. Do not infer a useful learning signal or monotonic energy saving from the existence of the control path.

Qualification requires analytical continuity/retargeting tests, historical compatibility checks, and matched native up/down transitions from exactly the same post-startup state, with charge/SOC reconciliation and repeat replay. The runner is `run_transition_acceleration_qualification`; results belong in a separate dated record. See [the transition and action contract](CONTINUOUS_TRANSITIONS.md) for equations, consumption coupling, R, and prospective additional controls.

### B. Define reproducible SOC coverage

Implement a training reset policy with an explicit initial-SOC schedule or distribution and saved sampling state. Specify the allowed range, stage progression, and selection seed before running it. Do not treat a chosen curriculum as a physical fact or assume the untested low-SOC speed boundary is known.

Archive the actual initial SOC of every episode. Reset SOC only at an episode boundary; retain battery state through all decisions and chunks of an episode. Confirm that all reset-dependent reward budgets and observations are consistent with the selected initial SOC.

Evaluation must include both:

- Matched, separately identified starting-SOC cases, using fixed conditions and seeds for comparisons between policies.
- Continuous operation beginning at 95% and 100%, with the battery carried forward until the declared mission/end condition. Define sufficient operation duration or repeated mission semantics to reach the SOC region being studied. A fresh high-SOC reset after every 320 m mission does not test natural depletion into that region.

### C. Qualify the action path and completion behavior

Run captured native speed transitions and representative low-SOC cases through the same action path that RL uses. Direct-speed plant runs and interval-mean electrical replays provide useful evidence but cannot replace this qualification.

Check body attitude, actual speed tracking, solver outcomes, drive limits, SOC reserve, and the exact accepted terminal state. Establish an explicit fall/invalid-state policy before training can exploit such states. Treat attitude thresholds as declared criteria until supported by evidence; telemetry alone is not a fall constraint.

Correct the active pack specification's all-SOC speed claim to match evidence. Current mean-power screening brackets a failure from a 35% start and a pass from a 50% start at 1.0 m/s; it does not establish a universal 35–40% transition boundary. Record voltage/current model extrapolation and distinguish a modeled limit from validated hardware capability.

### D. Specify the learning experiment and comparisons

Revisit the time/charge tradeoff on the current plant. The existing 0.34 time weight comes from an older configuration and must be labeled provisional until current matched comparisons support the selected study conditions. Compare speed and actual charge/energy explicitly before calling one preference best.

Set a training budget that can actually exceed warm-up and produce parameter updates. The 64-transition warm-up may span multiple episodes; 120 maximum decisions per episode is not a guarantee that an episode lasts that long or that a campaign learns useful behavior. Retain the distinction between an interface-learning smoke and mission-performance training.

Before expensive training, measure runtime and output size on the finalized configuration. Any acceleration of simulation must be checked against the reference numerical behavior; do not reduce physical energy or compress battery time to reduce wall time.

### E. Complete evidence capture

Archive matching test reports and full state histories for qualification runs. For each run, retain source revision and dirty status, configuration hash, seeds, initial conditions, command history, physical charge/energy totals, termination reason, and artifact checksums. Full capture must permit reconstruction of the dynamics and claims made from the run.

## Promotion boundaries

- Passing gap 1 means the decision-timing change is implemented and its bounded probe is recorded. It does not mean all gates above are complete.
- A pilot training run is appropriate after the action contract, SOC reset policy, reward/terminal semantics, and representative native checks are fixed and archived. It remains an experiment, not evidence of a useful learned policy.
- A policy-performance claim requires matched evaluation against fixed-speed baselines, different initial SOCs, continuous high-SOC depletion, and multiple declared seeds/scenarios. Report unsuccessful runs and runtime alongside successful outcomes.
- Hardware feasibility requires further mechanical and electrical validation. It is outside the current reduced-model training-readiness claim.
