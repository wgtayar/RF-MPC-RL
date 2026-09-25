# Versioned Battery Feedback

Future Phase-3 runs can explicitly select the timestamp/current alignment fix:

```matlab
bundle.cfg.BATTERY.feedback_version = 'battery_feedback_timestamp_aligned_v2';
```

Missing feedback_version, or explicit battery_feedback_legacy_prefix_v1, preserves
the historical evaluate_battery_feedback implementation. No protected MAT file,
setup default, old dataset, SOC trace or label is rewritten. The captured environment
validates the version before creating its dataset, records it in configuration and
manifest metadata, and requires identical configuration for saved-decision resume.

The battery model returns a subset of its input timestamps after filtering SOC
rows. Legacy feedback paired those returned time/voltage rows with a prefix of the
original current trace. The new version selects pack AND cell current by exact
returned-timestamp membership. It neither interpolates, snaps approximate times,
truncates mismatched outputs nor assumes that only the first row was removed.
The diagnostic second output of phase3battery.evaluateAligned records original
history and decimated indices. Battery state structure remains unchanged.

The same model_battery, estimateSOC, parameter spreadsheet, full-history replay,
decimation, absolute-current convention, initial SOC and pack sizing are retained.
Previous battery state is not used to initialize replay: doing so would double
count history. Invalid time ordering/subsets or inconsistent shapes fail closed.
This correction does not establish hardware SOC accuracy or calibrate safety.

The prior training-only battery impact study reproduced176 archived legacy states
and measured a maximum0.164801023 percentage-point SOC difference under alignment,
with no sampled decision-boundary terminal changes. That does not prove unchanged
terminals at all chunk boundaries or under a new policy. New bounded runtime
checks are separate evidence, not a rerun/relabeling of historical experiments.

Use the explicit aligned version for future battery-corrected experimental runs,
but do not start robot RL or promote a reward/safety policy from this fix alone.
The frozen6321cc9 full-state predictor contract is historical and deliberately
rejects changed source. It must not be relabeled as compatible with this new
runtime; any new predictor/source binding requires its own audited version.
