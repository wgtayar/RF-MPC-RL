# Frozen simulated pack — 20S1P

New `setup_RL_MPC` configurations use `beyond_torque_20s1p_p95_20261103_v1`: 20 series × 1 parallel, 2 Ah/cell, 2 Ah pack, 74 V reported nominal voltage. The ideal simulated drive must support 83.632 V at full charge. Pack counts and cell capacity are locked at initialization and remain unchanged across RL episodes. Initial SOC remains an experiment setting.

The original `manuscriptConfig` and unversioned energy configurations preserve historical behavior. No raw dataset or fitted cell parameter is replaced.

## Evidence and decision

One complete governed-fast requested trajectory: 320.004 m in 506.52 s, 50,652 successful QPs. Voltage enforcement was disabled only during requested-demand acquisition. Reconstruction reproduced every saved mechanical endpoint exactly, then evaluated 32 midpoint loads, an independent 16-grid, and both endpoints per 10 ms interval.

Resolved demand: 3.702233425 Wh; sampled required-voltage peak 67.510970006 V; sampled peak motor torque 0.803963939 Nm against the retained 3 Nm model limit. The 16/32 energy difference is 0.0938193%. These are sampled/discretized results, not a continuous peak certificate.

The [RC bound](PACK_SIZING_PROOF.md) gives Vlow=3.5132 V/cell and Ns=ceil(67.510970006/Vlow)=20. Its all-amplitude reference is 20S3P. The selected smaller parallel count is justified by actual replay, not by the 1 A all-amplitude theorem. Candidate parallel counts were checked in ascending order; the smallest passing both SOC scenarios was selected. No arbitrary safety multiplier or global mass-optimality claim is used.

| Initial SOC | P95 cell current (A) | Peak checked cell current (A) | Time above 1 A (s) | Final true SOC (%) | Minimum conservative voltage headroom (V) |
|---|---:|---:|---:|---:|---:|
| 25% | 0.757317107 | 2.192642317 | 12.791562 | 22.541846848 | 6.192314646 |
| 95% | 0.707805328 | 2.041306399 | 10.562812 | 92.703478496 | 11.647231140 |

The 95th percentile is the empirical inverse CDF with equal-duration midpoint weights, including startup. It is not a confidence interval. Motor phase currents are converted to simultaneous terminal watts before computing battery current. The 1 A threshold is the manuscript characterization amplitude (0.5 C × 2 Ah), not a discharge rating. All current tails remain in the root/voltage/torque/SOC checks and are explicitly unvalidated extrapolation.

## Runtime and scope

Both MPC consumers use the [resolved provider](RESOLVED_ENERGY_METHOD.md), enforce true 20% reserve at every battery halfstep, and preserve transactional failures. EKF transitions are composed through substep currents with one process-noise addition and one voltage update per original MPC interval. The battery/drive log distinguishes interval mean current/power, endpoint voltage, and sampled voltage envelope.

This freezes a new simulated electrical design. It does not establish compatibility with Mini Cheetah electronics, physical cell ratings, pack mass, thermal limits, insulation or motor accuracy at the higher speeds. The stipulated 5.5 kg mechanical plant and existing ideal-drive assumptions are unchanged. Unknown future RL actions are subject to runtime limits. Observation/reward/value and adverse-domain gates remain separate; no training promotion is implied.

## Reproduction and traceability

Selection source `116afc63955e346149d85782de3a658083b43af8`; resolved-demand source `0291fb84c3f86b6bbf123f003eee2ce073273956`; requested-motion source `35800e2`. The checked specification stores the full selection-report, raw resolved-input and cell-table hashes. See the monitor `runs/pack-sizing-20261103-a` for candidate records, exact manifests, negative/development evidence and local-only checksum indexes.

```matlab
run_frozen_pack_qualification(fullfile(fileparts(pwd), ...
    'BeyondTorque-Local-Artifacts','new-frozen-pack-check'))
```

This performs bounded native RL-interface qualification. The full-mission evidence is separately identified as an electrical replay of saved requested motion; it must not be mislabeled as a new fully coupled rollout or an RL learning result.
