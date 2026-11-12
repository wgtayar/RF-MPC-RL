function results = run_qualified_tests()
%RUN_QUALIFIED_TESTS Run the qualified energy-mission test set (deployment check).
% The full tests/ folder also holds legacy tests that need local-only files
% (e.g. rlEnv_MPC_R.mat); they are not part of this qualification.
% Run from a clean checkout: one pack-sizing test refuses a dirty tree.
bootstrap_RF_MPC_RL();
suites = {
    'testAttitudeTelemetry'
    'testBatteryManuscriptConfig'
    'testEnergyMissionDataset'
    'testEnergyMissionObjective'
    'testEnergyMissionPolicyEvaluation'
    'testEnergyMissionTiming'
    'testEnergyMissionTraining'
    'testFixedPackEnergy'
    'testFixedPackFailureHandling'
    'testFixedPackLegacyTiming'
    'testFixedPackMath'
    'testFixedPackPolicyEvaluation'
    'testFixedPackTargetCrossing'
    'testFrozenPack'
    'testInstantaneousDrive'
    'testMpcReplayTimeGrid'
    'testPackDemandReplay'
    'testPackSizingAnalysis'
    'testPackSizingBound'
    'testPhase3ActionSemantics'
    'testPhase3BatteryFeedback'
    'testPhase3TargetStop'
    'testPhase3TerminalSemantics'
    'testReferencePositionContinuity'
    'testResolvedEnergy'
    'testSizingDemandTrace'
    'testTrackedSwing'
    'testTransitionAcceleration'
    'testVelocityReference'
    'testGaitForecast'
    'testInitialSocCurriculum'
    'testRewardV3'
};
root = fileparts(mfilename('fullpath'));
results = [];
for k = 1:numel(suites)
    results = [results, runtests(fullfile(root,'tests',[suites{k} '.m']))]; %#ok<AGROW>
end
fprintf('QUALIFIED TESTS: %d passed, %d failed, %d incomplete, %d total\n', ...
    sum([results.Passed]), sum([results.Failed]), sum([results.Incomplete]), numel(results));
end
