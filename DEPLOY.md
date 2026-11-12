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

The training campaign is 3 time weights × 3 seeds (20260903, 20260904, 20260905). Launch each run in its own MATLAB process. The exact launch commands are added to this file together with the final reward version. **Do not start the campaign before that commit.**

A full 320 m episode takes about 35–45 minutes of wall-clock time on the development PC.
