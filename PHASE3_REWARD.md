# Opt-in Phase-3 Reward Candidate

The default `Phase3MpcEnvironment` still uses `reward_phase3_legacy_bridge_v1`.
The experimental coverage candidate requires both explicit options:

```matlab
options.reward_schema = 'reward_phase3_coverage_candidate_v1';
options.reward_policy = phase3reward.unitPolicy('unit_reference_experimental_v1');
```

This does not enable training or claim calibrated safety. The complete policy is
archived in the environment configuration, with `training_promoted=false`.
Only fixed-request, fixed-pack configurations are currently supported.

The unit-reference hypothesis combines schedule-delta pace, time-integrated
squared orientation/angular-velocity/tracking errors, modeled sampled charge,
and a completion bonus. Physical scales are unit references, not safety limits.
Action-change/acceleration regularization and recovered-event penalties are zero.
Numerical failure, unclassified solver failure, timeout, and battery terminals
receive no additional terminal penalty; they still lose unearned completion
credit and retain executed costs. Dynamic safety, mathematical infeasibility,
invalid state, and no-safe-action terminals receive an explicit experimental
unit loss. These terminal coefficients are not empirically calibrated.

Missing physical exposure is allowed only for an explicit invalid-state terminal:
keep known integrals, charge a named positive missing-duration cost, and remove
positive pace credit. This is not imputation or an upper bound on unknown cost.
Numerical failure is never relabeled as mathematical infeasibility.

`ExactStateDataset` reduces captured MPC rows per decision without changing QP
or plant equations. Charge booking carries the preceding committed sample across
decision boundaries. Startup absence and endpoint tails remain explicitly
unobserved; the current model is not a hardware energy measurement.
`validate_exact_state_dataset` reconstructs candidate reward/audit values and
checks the saved current history against captured samples. Legacy datasets retain
their previous validation path. Batch-quadrature parity has separate unit tests.

The namespaced kernels in `+phase3reward` are deployed copies of the historical
RL-MPC-Monitor candidate component/scale/scoring/exposure/charge contracts,
present at monitor commit `3ec949b3c2a3bd479a1a7a851561a4fa09f9bc2a`.
Historical analysis helpers are not rewritten or shadowed.
The motivating reward-family screen is documented there in
`docs/rl/REWARD_FAMILY_SCREEN.md`; its unit point was post-exploratory, not held-out
calibration. Observation sufficiency and empirical viability gates remain open.

Keep all new raw capture data outside Git in the configured local-only artifact
store. No new LFS uploads are permitted. Publish only small code, documentation,
indexes and checksums through the existing publication guards.
