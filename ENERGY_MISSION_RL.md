# Energy-aware mission interface (2026-11-04)

New `setup_RL_MPC()` calls `setup_energy_mission_rl()`. The frozen 20S1P plant, 2 Ah cell, resolved electrical integration, actuator screening, and true-SOC floor remain unchanged. This is the diagnostic baseline, not an approved physical battery design. Historical diagnostic runners explicitly request `setup_RL_MPC('legacy_frozen_v1')`; saved legacy actors cannot be transplanted into the new interface.

## Consumption and objective

Current and torque have different meanings. The motor model computes phase current from joint torque/gearing, shaft power and copper losses; the battery solves its 3RC circuit at that power. Booked pack current is pack charge divided by actual elapsed time. For the fixed identical-cell pack:

`Q_pack = integral(I_pack dt)` and `SOC_true = SOC_start - Q_pack/(3600*Np*C_cell_Ah)`.

The decision builder checks incremental charge against the physical battery accumulator. Reward uses that charge, never differences in the EKF SOC estimate. No load multiplier, invented auxiliary consumption, or shortened battery capacity is introduced. Auxiliary power is still zero and inverter efficiency one; results describe this modeled drive, not measured whole-robot energy.

The 87-feature observation retains executed motion/contact/action/solver history and adds estimated SOC/RC state, actual mean current, charge history, voltage/headroom, and a historical charge-per-meter estimate of remaining demand. The forecast uses only executed history, is unavailable at reset, and is not a safety bound. True SOC/RC state is not a policy input. Current is expressed in pack 1C units (2 A here) with an unclipped `asinh` transform; 1C is a unit, not a cell current rating. Old 45–65 A hinges and the 90 A normalization do not participate in the new reward or observation.

For elapsed-time limit `T`, target `D`, charge reference `Qref`, and nonnegative preference weights summing to one:

`r = delta clip(distance/D,0,1) - w_time*delta_t/T - w_charge*delta_Q/Qref + B*success - failure`.

`Qref` is the recorded 95%-start reference charge per meter times `D`, with its evidence hashes in `energy_mission_reference.json`. This is a normalization unit, not a forecast that consumption scales linearly. `Qbudget = 3600*Np*C_cell_Ah*(SOC_start-SOC_floor)` and `B = 2+w_time+w_charge*Qbudget/Qref`.

For a complete episode starting at zero progress with discount exactly one, bounded time/charge and exactly one terminal, progress telescopes. Every successful episode has return at least 3; every failed episode has return at most 0. Among successes of the same scenario and preference, maximizing return minimizes the stated time/charge cost. Backward motion cannot repeatedly earn progress. These bounds do not describe an isolated resumed suffix. No extra terminal SOC bonus double-counts consumed charge. Highest true final SOC is equivalent to lowest charge only for the same pack and initial SOC.

There is no arbitrary SOC trigger. Charge is costly from the start; reserve and remaining task state can inform learned behavior. Whether adaptation offers value must be demonstrated against strong fixed policies. The current 320 m reference consumes only 2.30 SOC percentage points from 95%; it does not demonstrate reserve-driven adaptation.

## Configuration and comparisons

`setup_energy_mission_rl(options)` accepts `initial_soc` (0.95 or 1), `target_distance_m`, `mission_duration_s`, `decision_interval_s`, `time_weight`, and `seed`. The charge weight is `1-time_weight`. Defaults preserve the 320 m/600 s mission and 50 s decision interval. Longer continuous missions are supported through explicit distance/time; their dynamics, consumption and depletion remain unvalidated until simulated. Battery state persists across decisions/chunks and resets only at an episode boundary.

The default equal weighting is provisional. The user's chosen decision method is to compare time and total charge/energy before selecting a tradeoff. Report successful-policy Pareto comparisons within identical missions, packs, and starting SOCs; do not compare raw returns across different weights or select a pack by lowering SOC artificially.

`run_energy_mission_comparison(outputRoot, options)` records matched native policy comparisons with exact snapshots, charges, energies, true/estimated SOC, reward terms and terminal causes. Its default 20 m/50 s, 5 s-decision scenarios are explicitly short integration/action-sensitivity tests, not full-mission evidence or the final mission design. `run_pack_series_diagnostic(inputRoot, outputRoot, options)` replays the existing full resolved demand on explicit unlocked diagnostic series-count candidates. It neither changes nor selects the locked pack. Smaller capacity requires a supported cell model; editing capacity alone while keeping the old resistance/OCV assumptions is not a qualified redesign.

## Training interface

`train_energy_mission_rl(outputRoot, options)` builds a fresh TD3 agent by default (`mode='build_only'`); `mode='train'` enables captured learning. Native 87-feature observations, paired reward schema and exact target stopping are enforced. Normalized [-1,1] actions map to the existing physical bounds before the governor; exploration therefore respects the very small R-action scale. All candidate/applied actions are recorded. The finite-horizon objective uses discount one.

Twin critics, delayed actor updates and explicit target/exploration noise are supported by the installed MATLAB API. The starting hyperparameters are reproducible, provisional settings, not empirically tuned performance. Short optimizer smoke tests establish that data reach the networks and update parameters; they do not establish a useful controller. Legacy Agent74/75 stay untouched; incompatible warm starts are rejected. The compatibility `train_RL_MPC` entry creates a fresh captured run instead of loading an old worker MAT. Ordinary policy evaluation uses a fresh energy configuration; explicit `configuration_mode='saved_legacy'` is required to evaluate a saved historical bundle.

All raw datasets/checkpoints belong in the non-Git sibling BeyondTorque-Local-Artifacts. Each native run captures source SHA/dirty state, full configuration/hash, schemas, seed, effective optimizer/noise settings and outcome. `run_energy_mission_qualification` tests legacy physical regressions, new reward/observation/action math, and a two-transition native optimizer smoke. Numerical qualification does not validate hardware, a fall envelope, real-time operation, learning quality, or pack suitability.
