# Working battery specification

The investigator approved the supplied manuscript, *Toward Energy-Efficient
Autonomous Mobile Robots: Integrated Battery Modeling and EKF-Based SOC
Estimation*, by Nancy Zayour and Nagham El Ghossein, as the working specification
on 2026-10-22. This unblocks simulation development without requesting another
electrical-team handoff first. It does not manufacture new validation evidence.

## Selected configuration

New `setup_RL_MPC` configurations use `phase3battery.manuscriptConfig()`:

- Existing experimental `our params.xlsx`, unchanged; cell nominal capacity 2 Ah.
- Manuscript EKF Q = diag([1e-5, 1e-3, 1e-3, 1e-3]), measurement variance 1e-3,
  and initial covariance diag([1e-4, 1e-3, 1e-3, 1e-3]).
- Timestamp-aligned feedback, already available as an explicit Phase-3 version.
- Unfiltered simulated terminal voltage. The manuscript does not specify an
  extra voltage low-pass stage; the historical lowpass can introduce endpoint
  bias on short traces. Select `voltage_filter='legacy_lowpass'` explicitly to
  reproduce that behavior; absent options keep it for historical compatibility.
- Existing 4s6p arrangement, 95% initial SOC and 20% mission floor. The 12 V
  field remains the legacy optional-sizing target, not measured terminal voltage.

Historical configurations with no `model_options` retain the old Q SOC entry
1e-7. Missing `feedback_version` retains legacy alignment. Existing MAT files,
archived states, policy results and spreadsheet values are not overwritten.
Running setup is an explicit new-configuration operation, not archive migration.

## Replaceable model inputs

`cfg.BATTERY.model_options.parameter_source` accepts a workbook path or a MATLAB
table with SOC, OCV, R0, R1, C1, R2, C2, R3 and C3 columns. SOC uses fractions,
OCV volts, resistance ohms, capacitance farads. Rows must be ordered and finite.
Both model and EKF consume this same source. Change `C_nom_Ah`, pack counts and
the covariance settings in the configuration for another assumed cell/pack.
Keep the configured table's SOC coverage consistent with the model's unchanged
20--100% domain and estimator's 10--100% clamp; no new extrapolation of OCV is
introduced. A new profile requires an explicit provenance/version decision.

The optional trailing model-options argument is also accepted by `model_battery`
and `estimateSOC`. No current clipping, new battery physics or noise injection
is introduced. The reported RMSE is metadata, not an EKF covariance setting.

## Accepted working assumptions and remaining limitations

| Item | Working interpretation |
| --- | --- |
| Cell chemistry | Manuscript reports LFP and 3.7 V nominal; identity remains unresolved. Preserve the empirical OCV table including 4.1816 V at full SOC rather than replacing it with a generic LFP curve. |
| Characterization | Reported room-temperature BioLogic SP-300 test with 10 A booster; 0.5C discharge for 360 s, then 1800 s rest, in 5% SOC increments. |
| Parameter fit | Reported optimization of experimental pulse responses; 3RC voltage RMSE 0.0276 V versus 2RC 0.0285 V. |
| EKF validation | Reported voltage RMSE 0.0033 V and SOC RMSE 0.0129 against Coulomb counting. Investigator selects fraction units: 1.29 percentage points. |
| Rest interval | Accept the reported laboratory procedure as the working basis. Complete equilibrium at each point is not independently established; no rest-data correction is applied. |
| Higher current | Permit extrapolation from available data for this simulated proof of concept. Accuracy outside the measured current/temperature range is unestablished. C is a capacity-normalized rate, not coulombs; at 44 A pack current, 6p and 2 Ah cells imply about 3.67C per cell. |
| Validation scope | Laboratory validation is reported by the authors. Raw datasets, fit/validation separation, exact cell identity and script/version linkage have not been reconciled in this repository. |
| Robot energy | These assumptions do not validate the existing motor-current-to-pack-current proxy, braking/regen, actuator losses or learned energy savings. Those remain separate architectural work. |

The intended system is a defined simulated quadruped, not a reconstruction of
one physical robot. Component parameters may come from different documented
platforms, with consistent units, mechanics and electrical boundaries. SOC is
in scope; SOH is deferred. Reassessing the speed cap with position-continuous
references is approved planning work, not implemented by this change.

## Evidence provenance

The supplied manuscript attachment SHA256 is recorded in the companion monitor
note. Its laboratory metrics are author-reported values, not measurements rerun
by this change. Software regression results are documented separately in the PR.
