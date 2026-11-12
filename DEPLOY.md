# Deploying on another machine (e.g. the university PC)

## 1. Requirements

- MATLAB **R2026a** (26.1). Other releases are untested.
- Toolboxes:
  - Optimization Toolbox (`quadprog`)
  - Reinforcement Learning Toolbox (TD3)
  - Deep Learning Toolbox (network layers)
  - Control System Toolbox
- `git` on the system `PATH`. Runs record source and monitor commit SHAs through it.
- Memory: about 1.5–2 GB per MATLAB process. Each training seed runs in its own process.

## 2. Folder layout

Every run writes the monitor repository's commit into its manifest, so the two repositories must be **side by side** under one parent folder:

```
<parent>/
  RF-MPC-RL/                      source (this repository)
  RL-MPC-Monitor/                 research record (must be a git checkout)
  BeyondTorque-Local-Artifacts/   raw outputs (plain folder, never committed)
```

```bash
git clone -b review/tracked-swing-18650-pack git@github.com:wgtayar/RF-MPC-RL.git
git clone -b review/fixed-pack-energy git@github.com:wgtayar/RL-MPC-Monitor.git
mkdir BeyondTorque-Local-Artifacts
```

Training and evaluation refuse to run on a dirty source tree and refuse to write outputs inside `RF-MPC-RL`. Always pass an output folder under `BeyondTorque-Local-Artifacts`.

## 3. Check the installation

From MATLAB, inside `RF-MPC-RL`:

```matlab
results = run_qualified_tests();   % expect all qualified tests to pass
```

## 4. Runs

The training and evaluation entry points are:

| Purpose | Function |
|---|---|
| Training (one seed, one weight) | `train_energy_mission_rl(outputRoot, options)` |
| SOC-grid evaluation of a checkpoint | `run_energy_mission_soc_grid(trainingRoot, outputRoot)` |
| Fixed-speed baselines on the grid | `run_energy_mission_comparison(outputRoot, options)` |
| Continuous depletion | `run_energy_mission_continuous_depletion(policy, outputRoot, options)` |

## 5. Training campaign (reward v3)

The campaign trains three time weights (`w_time` = 0.3, 0.63, 0.8) × three seeds (20260903, 20260904, 20260905), nine runs in total. Each run samples a new start SOC uniformly in 30–100% for every episode.

Launch each run as its own MATLAB process from inside `RF-MPC-RL`. Example for `w_time = 0.3`, seed 20260903, with `N` episodes:

```bash
matlab -batch "maxNumCompThreads(1); train_energy_mission_rl(fullfile('..','BeyondTorque-Local-Artifacts','train-w030-s20260903'), struct('mode','train','seed',20260903,'max_episodes',N,'setup_options',struct('time_weight',0.3)))"
```

For the other runs, change `time_weight` (0.3 / 0.63 / 0.8), `seed`, and the output folder name (`train-w063-s20260904`, and so on).

**Pilot first:** run one job (`w_time = 0.63`, seed 20260903) before the other eight. Check that `kappa_cumulative` at completed episodes stays within about [0, 1]. It is stored in each decision record under `reward_info.standardized_monitor`. If it does not, the 84.54 As charge span must be revised before the main runs. The episode budget `N` is decided from the pilot.

Practical notes:
- The simulator runs on one CPU core: about 3.7–4× slower than simulated time per process on the development laptop (Ryzen 7 5825U). A full 320 m episode at 0.8 m/s (401 s simulated) therefore takes about 25 minutes for a process running alone, and about 35–45 minutes with three processes sharing the laptop.
- Single-threaded MATLAB (`maxNumCompThreads(1)`) was slightly faster than 8 threads in this test (3.7× vs 4.0×). Run one training process per physical core; do not run more processes than physical cores.
- A GPU does not speed this up. Almost all time is spent in the CPU simulation (MPC quadratic program, rigid-body integration and the electrical model), which is made of many small sequential computations. The actor and critic networks (two layers of 128 units) are too small for a GPU to matter.
- A checkpoint is saved after every episode under `<output>/training/checkpoints`.
- An interrupted run cannot be resumed automatically; it must be restarted.
- Memory is about 1.5–2 GB per process.

## 6. Evaluation after training

```matlab
% Fixed-start SOC grid (30/35/45/60/80/95/100%) for one trained run
run_energy_mission_soc_grid('../BeyondTorque-Local-Artifacts/train-w063-s20260903', ...
    '../BeyondTorque-Local-Artifacts/eval-grid-w063-s20260903');

% Continuous depletion from 95% and 100% (policy, then a fixed-speed baseline)
run_energy_mission_continuous_depletion('../BeyondTorque-Local-Artifacts/train-w063-s20260903', ...
    '../BeyondTorque-Local-Artifacts/depl95-w063-s20260903', struct('initial_soc',.95));
run_energy_mission_continuous_depletion(struct('fixed_speed_m_s',.8), ...
    '../BeyondTorque-Local-Artifacts/depl95-fixed080', struct('initial_soc',.95));

% Fixed-speed baselines on the SOC grid (full 320 m missions)
run_energy_mission_comparison('../BeyondTorque-Local-Artifacts/baselines-grid', struct( ...
    'setup_options',struct('target_distance_m',320,'mission_duration_s',600,'decision_interval_s',5), ...
    'speeds_m_s',[.5333 .8 1.0],'initial_soc',[.30 .35 .45 .60 .80 .95 1]));
```
