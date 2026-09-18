classdef testPhase3RewardCandidate < matlab.unittest.TestCase
    properties
        Bundle
        Metadata
        Folder
    end
    properties (TestParameter)
        exposureCase = {'full','partial','failed','zero','cold','gap','invalid'}
    end
    methods (TestMethodSetup)
        function configure(testCase)
            source = bootstrap_RF_MPC_RL();
            testCase.Bundle = load(fullfile(source,'rlEnv_MPC_R.mat'), ...
                'cfg','initial_R','lower_abs','upper_abs');
            testCase.Bundle.cfg.RANDOMIZE_REQUEST = false;
            testCase.Bundle.cfg.BATTERY.use_pack_sizing = false;
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = fullfile(fixture.Folder,'dataset');
            testCase.Metadata = struct('run_id','candidate_reward_unit','run_type','unit_test', ...
                'monitor_root',source,'seed',1,'source_policy','no_mpc_rollout');
        end
    end
    methods (Test)
        function legacyDefaultsStayUnchanged(testCase)
            env = Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata);
            testCase.verifyEqual(env.Config.PHASE3.reward_version,'reward_phase3_legacy_bridge_v1');
            testCase.verifyFalse(isfield(env.Options,'reward_schema'));
            testCase.verifyFalse(isfield(env.Options,'reward_policy'));
            close(env,'unit_test_complete');
        end
        function candidateRequiresExplicitPolicy(testCase)
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata, ...
                struct('reward_schema','reward_phase3_coverage_candidate_v1')), ...
                'Phase3MpcEnvironment:RewardPolicy');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function policyWithoutOptInRejected(testCase)
            options = localOptions();
            options = rmfield(options,'reward_schema');
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata,options), ...
                'Phase3MpcEnvironment:RewardPolicy');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function abbreviatedSchemaRejected(testCase)
            options = localOptions();
            options.reward_schema = 'reward_phase3_coverage';
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata,options), ...
                'Phase3MpcEnvironment:RewardSchema');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function explicitPolicyIsArchivedWithoutTrainingPromotion(testCase)
            options = localOptions();
            env = Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata,options);
            testCase.verifyEqual(env.Config.PHASE3.options.reward_policy,options.reward_policy);
            testCase.verifyEqual(env.Writer.Manifest.reward_version,options.reward_schema);
            testCase.verifyEqual(env.Config.PHASE3.environment_version,'phase3_captured_environment_coverage_reward_v1');
            testCase.verifyFalse(env.Config.PHASE3.training_promoted);
            testCase.verifyEqual(env.Writer.RowCount,0);
            close(env,'unit_test_complete');
            report = validate_exact_state_dataset(testCase.Folder);
            testCase.verifyEqual(report.rewards_reconstructed,0);
        end
        function randomRequestsRejectedBeforeCapture(testCase)
            bundle = testCase.Bundle;
            bundle.cfg.RANDOMIZE_REQUEST = true;
            testCase.verifyError(@() Phase3MpcEnvironment(bundle,testCase.Folder,testCase.Metadata,localOptions()), ...
                'candidateReward:Configuration');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function packSizingRejectedBeforeCapture(testCase)
            bundle = testCase.Bundle;
            bundle.cfg.BATTERY.use_pack_sizing = true;
            testCase.verifyError(@() Phase3MpcEnvironment(bundle,testCase.Folder,testCase.Metadata,localOptions()), ...
                'candidateReward:Configuration');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function unnamedPolicyRejected(testCase)
            options = localOptions();
            options.reward_policy.policy_id = ["one","two"];
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata,options), ...
                'phase3reward:Policy');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function promotedPolicyRejected(testCase)
            options = localOptions();
            options.reward_policy.training_promoted = true;
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata,options), ...
                'phase3reward:Policy');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function streamingMatchesIndependentBatchQuadrature(testCase,exposureCase)
            [rows,steps] = localRows(exposureCase);
            actual = localReduce(rows);
            expected = phase3reward.fromSteps(steps,0.01);
            testCase.verifyEqual(rmfield(actual,'scope'),rmfield(expected,'scope'),AbsTol=1e-13);
        end
        function streamingRejectsRowGap(testCase)
            rows = localRows('full');
            a = phase3reward.accumulate([],rows{1},0.01);
            testCase.verifyError(@() phase3reward.accumulate(a,rows{3},0.01),'phase3reward:Continuity');
        end
        function streamingRejectsWrongSampleTime(testCase)
            rows = localRows('full');
            row = rows{1};
            row.current_sample_time = 0.005;
            testCase.verifyError(@() phase3reward.accumulate([],row,0.01),'phase3reward:Current');
        end
        function streamingRejectsInconsistentFlags(testCase)
            rows = localRows('full');
            row = rows{1};
            row.solver.success = false;
            testCase.verifyError(@() phase3reward.accumulate([],row,0.01),'phase3reward:Flags');
        end
        function streamingRejectsOverflow(testCase)
            rows = localRows('full');
            row = rows{1};
            row.health_before.components.orientation_error_rad = realmax;
            testCase.verifyError(@() phase3reward.accumulate([],row,0.01),'phase3reward:Overflow');
        end
        function nativeEmptyCurrentHistoryWorks(testCase)
            [w,x,e,b,a,p] = localCandidate('cold');
            [reward,info] = compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p);
            testCase.verifyTrue(isfinite(reward));
            testCase.verifyEqual(info.charge_audit.booked_charge_As,0);
            testCase.verifyFalse(info.charge_audit.any_pair_booked);
            testCase.verifyFalse(info.exposure.current_pair_observed);
        end
        function warmHistoryBooksBoundaryExactlyOnce(testCase)
            [w,x,e,b,a,p] = localCandidate('full');
            b.t = 0.05;
            a.t = 0.1;
            b.current_time = (0:4)'*0.01;
            b.current_total = 10*ones(5,1);
            a.current_time = (0:9)'*0.01;
            a.current_total = 10*ones(10,1);
            w.charge_As = 0.5;
            [~,info] = compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p);
            testCase.verifyEqual(info.charge_audit.boundary_charge_As,0.1,AbsTol=1e-14);
            testCase.verifyEqual(info.charge_audit.booked_charge_As,0.5,AbsTol=1e-14);
        end
        function modifiedHistoryRejected(testCase)
            [w,x,e,b,a,p] = localCandidate('full');
            b.current_time = 0;
            b.current_total = 11;
            testCase.verifyError(@() compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p), ...
                'phase3reward:History');
        end
        function inconsistentChargeRejected(testCase)
            [w,x,e,b,a,p] = localCandidate('full');
            w.charge_As = 0;
            testCase.verifyError(@() compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p), ...
                'phase3reward:Accounting');
        end
        function invalidTerminalIsFiniteWithoutPositivePace(testCase)
            [w,x,e,b,a,p] = localCandidate('partial');
            w.terminal_reason = 'invalid_state';
            w.distance_end_m = 100;
            [reward,info] = compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p);
            testCase.verifyTrue(isfinite(reward));
            testCase.verifyLessThan(reward,0);
            testCase.verifyLessThanOrEqual(info.pace_contribution,0);
            testCase.verifyFalse(info.physical_costs_complete);
            testCase.verifyGreaterThan(sum(info.weighted_missing_exposure_costs),0);
        end
        function missingHealthCannotRelabelNumericalFailure(testCase)
            [w,x,e,b,a,p] = localCandidate('partial');
            w.terminal_reason = 'numerical_solver_failure_unrecovered';
            testCase.verifyError(@() compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p), ...
                'coverageReward:MissingNonterminal');
        end
        function zeroTimeNumericalFailureKeepsItsIdentity(testCase)
            [w,x,e,b,a,p] = localCandidate('zero');
            w.terminal_reason = 'numerical_solver_failure_unrecovered';
            [reward,info] = compute_phase3_reward_candidate(w,x,e,b,a,testCase.Bundle.cfg,p);
            testCase.verifyTrue(isfinite(reward));
            testCase.verifyEqual(info.actual_terminal_reason,"numerical_solver_failure_unrecovered");
            testCase.verifyEqual(info.terminal_contribution,0);
            testCase.verifyFalse(info.training_promoted);
        end
    end
end

function options = localOptions()
    options = struct('reward_schema','reward_phase3_coverage_candidate_v1', ...
        'reward_policy',phase3reward.unitPolicy('unit_reference_experimental_v1'));
end

function [rows,steps] = localRows(kind)
    n = 5;
    steps = table((0:n-1)'*0.01,(1:n)'*0.01,true(n,1),true(n,1), ...
        true(n,1),(0:n-1)'*0.01,10*ones(n,1),ones(n,1),ones(n,1), ...
        2*ones(n,1),2*ones(n,1),3*ones(n,1),3*ones(n,1), ...
        VariableNames={'time_before','time_after','integrated','success', ...
        'current_sample_committed','current_sample_time','current_sample_A', ...
        'orientation_before','orientation_after','omega_before','omega_after', ...
        'velocity_error_before','velocity_error_after'});
    switch kind
        case 'partial'
            steps.omega_after(end) = NaN;
        case 'invalid'
            steps{end,{'orientation_after','omega_after','velocity_error_after'}} = NaN;
        case {'failed','zero'}
            if strcmp(kind,'zero')
                steps = steps(1,:);
            end
            steps.integrated(end) = false;
            steps.success(end) = false;
            steps.current_sample_committed(end) = false;
            steps.time_after(end) = steps.time_before(end);
        case 'cold'
            steps.current_sample_committed(:) = false;
        case 'gap'
            steps.current_sample_committed(3) = false;
    end
    rows = cell(height(steps),1);
    for k = 1:height(steps)
        before = struct('valid',true,'components',struct( ...
            'orientation_error_rad',steps.orientation_before(k),'angular_velocity_norm',steps.omega_before(k)), ...
            'linear_velocity_error',steps.velocity_error_before(k));
        after = struct('valid',~(strcmp(kind,'invalid') && k==height(steps)), ...
            'components',struct('orientation_error_rad',steps.orientation_after(k), ...
            'angular_velocity_norm',steps.omega_after(k)),'linear_velocity_error',steps.velocity_error_after(k));
        rows{k} = struct('row_id',k,'time_before',steps.time_before(k),'time_after',steps.time_after(k), ...
            'integrated',steps.integrated(k),'solver',struct('success',steps.success(k)), ...
            'current_sample_committed',steps.current_sample_committed(k), ...
            'current_sample_time',steps.current_sample_time(k),'current_sample_A',steps.current_sample_A(k), ...
            'health_before',before,'health_after',after);
    end
end

function exposure = localReduce(rows)
    a = [];
    for k = 1:numel(rows)
        a = phase3reward.accumulate(a,rows{k},0.01);
    end
    exposure = phase3reward.finishExposure(a);
end

function [w,x,e,b,a,p] = localCandidate(kind)
    [rows,steps] = localRows(kind);
    e = localReduce(rows);
    b = struct('t',0,'gait',0,'current_time',[],'current_total',[]);
    a = b;
    a.t = steps.time_after(end);
    a.current_time = steps.current_sample_time(steps.current_sample_committed);
    a.current_total = steps.current_sample_A(steps.current_sample_committed);
    charge = 0;
    if numel(a.current_time)>1
        charge = trapz(a.current_time,abs(a.current_total));
    end
    w = struct('tracking_error_mean',1,'control_effort_mean',1,'Ieq_window',30, ...
        'soc_start_pct',95,'soc_end_pct',94,'lag_frac',0,'time_frac',0,'progress_frac',0, ...
        'distance_start_m',0,'distance_end_m',0,'window_distance_m',0,'v_exec',0.5,'a_exec',0.25, ...
        'delta_v_exec',0,'delta_gamma_v',0,'dR2',0,'state_norm_proxy',0,'com_speed_mag',0.5, ...
        'terminal_reason','','battery',struct('margin_norm',0.94),'recovered_solver_events',0, ...
        'duration_s',a.t,'charge_As',charge);
    x = struct('R_before',ones(3,1),'R_applied',ones(3,1),'delta_gamma_v_applied',0,'delta_v_exec',0,'a_exec',0.25);
    p = phase3reward.unitPolicy('unit_reference_experimental_v1');
end
