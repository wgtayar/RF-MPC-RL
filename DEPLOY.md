# Deploying on another machine (e.g. the university PC)

## 1. Requirements

- MATLAB **R2025b or newer**. Training and `run_qualified_tests` stop with an error on older releases.
  - Development and the reward constants (charge floor 637.573 As, span 78.17 As) used R2026a.
  - R2025b is the university PC's release. No R2026a-only function is used: the TD3, `rlTrainingOptions`, `dlnetwork` layer, `quadprog` and `ode45` calls all exist in R2025b.
  - The release is written into every run manifest (`matlab_version`). Solver results can differ between releases in the last digits, so `run_qualified_tests` on the university PC is the compatibility check. The pilot's κ range shows whether the charge constants still fit.
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
git clone git@github.com:wgtayar/RF-MPC-RL.git
git clone git@github.com:wgtayar/RL-MPC-Monitor.git
mkdir BeyondTorque-Local-Artifacts
```

Training and evaluation refuse to run on a dirty source tree and refuse to write outputs inside `RF-MPC-RL`. Always pass an output folder under `BeyondTorque-Local-Artifacts`.

### University lab PC (Windows, MATLAB R2025b): everything on D:

C: is nearly full. Since 2026-09-16 the canonical workspace has been on D:, and the old C: copy is a stale backup that must not be used (see `RL-MPC-Monitor/docs/project/WORKSPACE_D_TRANSITION.md`). The existing checkouts are:

```
D:\Energy Efficiency in Legged Robots\WG-Engineering\URE303\
  RF-MPC-RL\                      (HTTPS origin, last used on branch codex/phase-3-healthy-rl)
  RL-MPC-Monitor\
  BeyondTorque-Local-Artifacts\   (outside Git; has the root marker)
```

- **Do not clone again.** Update the existing D: checkouts: `git fetch origin`, `git switch main` (or `git switch -c main --track origin/main`), `git pull --ff-only`. Keep the local hook setting (`core.hooksPath`).
- **Every path must be on D:.** That covers the MATLAB current folder, run outputs, the `-logfile`, and the temporary folder. Set `TEMP` and `TMP` to a D: folder for each training process (see section 5).
- **`git` must be on the `PATH` of the launched MATLAB process**, not only in the editor's MATLAB session. Without it, a run stops at startup with "Cannot collect Git provenance" (pilot attempt of 2026-11-13). The launch command in section 5 checks this.
- **Check free space on D: before each run.** Full capture uses about 0.4 GB per 320 m episode (section 5).
- **Before a long run, stop Windows from sleeping or restarting the PC.** Set power to "never sleep", and pause Windows Update or set active hours. A forced restart kills the run, and runs cannot be resumed.

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

On the Windows lab PC, run in PowerShell from the D: checkout. This starts an independent MATLAB process that keeps running if the editor or terminal closes:

```powershell
$W = 'D:\Energy Efficiency in Legged Robots\WG-Engineering\URE303'
$Run = 'train-w063-s20260903-pilot'
$Out = "$W\BeyondTorque-Local-Artifacts\$Run"
New-Item -ItemType Directory -Force "$W\BeyondTorque-Local-Artifacts\_tmp\$Run", "$W\BeyondTorque-Local-Artifacts\_logs" | Out-Null
$env:TEMP = "$W\BeyondTorque-Local-Artifacts\_tmp\$Run"; $env:TMP = $env:TEMP
# git must be on PATH for the new process (runs record source and monitor SHAs).
# $GitDir is the folder holding git.exe; in MATLAB, system('where git') shows it.
$GitDir = 'C:\Program Files\Git\cmd'
if (-not (Get-Command git -ErrorAction SilentlyContinue)) { $env:Path = "$GitDir;$env:Path" }
git --version   # must print a version; otherwise stop
Set-Location "$W\RF-MPC-RL"
Start-Process matlab -ArgumentList @('-logfile', "`"$W\BeyondTorque-Local-Artifacts\_logs\$Run.log`"", '-batch',
  "`"maxNumCompThreads(1); train_energy_mission_rl('$Out', struct('mode','train','seed',20260903,'max_episodes',20,'setup_options',struct('time_weight',0.63)))`"")
```

`$Out` must not exist yet; runs never overwrite. While a run is going, do not `git pull`, switch branches or edit files in `RF-MPC-RL`. The run checks at the end that the source is unchanged, and otherwise ends as a failure.

For the other runs, change `time_weight` (0.3 / 0.63 / 0.8), `seed`, and the output folder name (`train-w063-s20260904`, and so on).

**Pilot first:** run one job (`w_time = 0.63`, seed 20260903) before the other eight. Before setting the episode budget `N` for the main runs, the pilot must show:
- training well beyond TD3's 64-transition warm-up, with the actor and both critic parameters actually changing (`report.json`: `actor_parameters_changed`, `critic_parameters_changed`);
- `kappa_cumulative` at completed episodes staying within about [0, 1]. It is stored in each decision record under `reward_info.standardized_monitor`. If it does not, the 78.17 As charge span must be revised before the main runs;
- how episodes end (completion versus each failure type), wall-clock time per episode, and disk usage per episode.

Practical notes:
- The simulator runs on one CPU core: about 3.7–4× slower than simulated time per process on the development laptop (Ryzen 7 5825U). A full 320 m episode at 0.8 m/s (401 s simulated) therefore takes about 25 minutes for a process running alone, and about 35–45 minutes with three processes sharing the laptop.
- Single-threaded MATLAB (`maxNumCompThreads(1)`) was slightly faster than 8 threads in this test (3.7× vs 4.0×). Run one training process per physical core; do not run more processes than physical cores.
- A GPU does not speed this up. Almost all time is spent in the CPU simulation (MPC quadratic program, rigid-body integration and the electrical model), which is made of many small sequential computations. The actor and critic networks (two layers of 128 units) are too small for a GPU to matter.
### What is logged (nothing from training is discarded)

Measured with a 2-episode × 3-decision training run on `b7c3739` (validation passed: 3000 rows, 6 rewards and windows reconstructed):

| Where | What | When written |
|---|---|---|
| `mpc_steps/` | every 10 ms MPC step: state, command, forces, solver result, battery and drive quantities | continuously, in blocks of about 256 rows |
| `failures/qp_row_*.mat` | complete QP problems: every solver failure, every dynamic event, the first step, and every 100th step as an audit sample (the folder name does not mean failure) | when it occurs |
| `decisions/` | each 5 s RL decision: observation, raw and executed action, reward and its terms (`reward_info.standardized_monitor` holds τ and κ) | after each decision |
| `episodes/` | start and end record of each episode, including the sampled start SOC | at episode start and end |
| `snapshots/`, `normalized_actions/`, `manifest/` | restore snapshots, action log, source and monitor SHAs, MATLAB release, configuration hash | during the run |
| `training/checkpoints/AgentN.mat` | actor, critics and optimizer state after episode N, plus `savedAgentResult` (all episode rewards, steps and Q0 so far) | after every episode |
| `training/console.log`, `_logs/<run>.log` | one line per episode (reward, steps, Q0); full MATLAB output | continuously |
| `training/final_agent.mat`, `report.json`, `validation.json`, `artifact_sha256.csv` | final agent and statistics, parameter-change checks, dataset validation, hash of every file | at the end |
| `training/failure.json`, `interrupted_agent.mat` | error, stack and the agent at the moment of a MATLAB error | on error |

- **Not saved in checkpoints: the TD3 replay buffer** (checkpoint buffer length was 0). Every transition is still in `decisions/`, so the buffer can be rebuilt from there. No experience is lost, but a crashed run cannot simply continue; it must be restarted.
- **Size:** captures are written as compressed MAT files (`-v7`), with every row and value kept. That is about 1 MB per simulated second; it was 3 MB with the earlier uncompressed `-v7.3` files. `mpc_steps` is 87% of it, and the QP samples are 3%. That gives about 0.4 GB per full 320 m episode at 0.8 m/s, about 8 GB for a 20-episode pilot, and about 0.4 TB per 1,000 episodes in total, plus about 0.33 MB of checkpoint per episode. Check this against free space on D: before setting the campaign budget. If space is still short, the next lever is the QP audit interval (`ExactStateDataset.AuditInterval`, now 100). Thinning the 10 ms rows would break reward reconstruction and validation.
- **After a run:** register its raw folder with `RL-MPC-Monitor/scripts/local_artifacts.py register` (see `RL-MPC-Monitor/docs/project/LOCAL_ARTIFACT_POLICY.md`). Commit only small summaries to the monitor, never raw MAT files.
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
