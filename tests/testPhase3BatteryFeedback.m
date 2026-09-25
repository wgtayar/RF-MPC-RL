classdef testPhase3BatteryFeedback < matlab.unittest.TestCase
    properties
        Config
    end
    methods (TestMethodSetup)
        function config(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(bootstrap_RF_MPC_RL()));
            testCase.Config = struct('SOC_init',0.95,'n_series',4,'n_parallel',6, ...
                'decim',10,'use_pack_sizing',false,'C_nom_Ah',2,'pack_voltage',12,'DoD',.8);
        end
    end
    methods (Test)
        function automaticZeroLoadKeepsLegacyPackConvention(testCase)
            cfg = testCase.Config; cfg.use_pack_sizing = true;
            times = (0:.01:2).'; currents = zeros(size(times));
            expected = evaluate_battery_feedback(times,currents,cfg);
            cfg.feedback_version = 'battery_feedback_timestamp_aligned_v2';
            [actual,audit] = phase3battery.evaluateAligned(times,currents,cfg);
            testCase.verifyEqual(actual,expected);
            testCase.verifyEqual(actual.n_parallel,0);
            testCase.verifyNotEmpty(actual.bms_input);
            testCase.verifyTrue(all(isfinite(actual.trace_metric)));
            testCase.verifyEqual(audit.effective_parallel_divisor,1);
            testCase.verifyEqual(evaluate_battery_feedback(times,currents,cfg),actual);
        end
        function fixedZeroParallelCountIsInvalid(testCase)
            cfg = testCase.Config; cfg.n_parallel = 0;
            testCase.verifyError(@()phase3battery.evaluateAligned((0:.01:2).',ones(201,1),cfg), ...
                'MATLAB:expectedPositive');
        end
        function nonemptyDispatchMatchesOriginalHistoryIndices(testCase)
            cfg = testCase.Config; cfg.feedback_version = 'battery_feedback_timestamp_aligned_v2';
            times = (0:.01:2.03).'; currents = 10+3*sin(7*times);
            [expected,audit] = phase3battery.evaluateAligned(times,currents,cfg);
            testCase.verifyEqual(evaluate_battery_feedback(times,currents,cfg),expected);
            testCase.verifyEqual(expected.bms_input.Time,times(audit.retained_history_indices));
            testCase.verifyEqual(expected.bms_input.PackCurrent,currents(audit.retained_history_indices));
            testCase.verifyEqual(audit.retained_history_indices(end),numel(times));
        end
        function packInputDoesNotReinitializeFullHistoryReplay(testCase)
            cfg = testCase.Config; cfg.feedback_version = 'battery_feedback_timestamp_aligned_v2';
            times = (0:.01:2).'; currents = 10+3*sin(7*times);
            a = evaluate_battery_feedback(times,currents,cfg,struct('soc_pct',20));
            b = evaluate_battery_feedback(times,currents,cfg,struct('soc_pct',95));
            testCase.verifyEqual(a,b);
        end
        function missingVersionIsHistorical(testCase)
            testCase.verifyEqual(phase3battery.version(testCase.Config),'battery_feedback_legacy_prefix_v1');
        end
        function explicitLegacyIsIdentical(testCase)
            expected = evaluate_battery_feedback([],[],testCase.Config);
            cfg = testCase.Config; cfg.feedback_version = 'battery_feedback_legacy_prefix_v1';
            testCase.verifyEqual(evaluate_battery_feedback([],[],cfg),expected);
        end
        function alignedDispatchAndEmptyHistory(testCase)
            cfg = testCase.Config; cfg.feedback_version = 'battery_feedback_timestamp_aligned_v2';
            [expected,audit] = phase3battery.evaluateAligned([],[],cfg);
            testCase.verifyEqual(evaluate_battery_feedback([],[],cfg),expected);
            testCase.verifyEqual(expected.soc_pct,95);
            testCase.verifyFalse(audit.interpolation_used || audit.pack_state_used);
        end
        function unknownVersionFailsBeforeHistoryWork(testCase)
            cfg = testCase.Config; cfg.feedback_version = 'aligned';
            testCase.verifyError(@()evaluate_battery_feedback([],[],cfg),'phase3battery:Version');
        end
        function versionVectorRejected(testCase)
            cfg = testCase.Config; cfg.feedback_version = ["a","b"];
            testCase.verifyError(@()phase3battery.version(cfg),'phase3battery:Version');
        end
        function emptyVersionRejected(testCase)
            cfg = testCase.Config; cfg.feedback_version = '';
            testCase.verifyError(@()phase3battery.version(cfg),'phase3battery:Version');
        end
        function selectsRemovedPrefix(testCase)
            [pack,cellCurrent,index] = phase3battery.alignSamples((0:3).',(1:4).',(1:3).',2);
            testCase.verifyEqual(pack,[2;3;4]);
            testCase.verifyEqual(cellCurrent,[1;1.5;2]);
            testCase.verifyEqual(index,[2;3;4]);
        end
        function arbitrarySubsetNotPrefix(testCase)
            [pack,~,index] = phase3battery.alignSamples((0:5).',(10:15).',[0;2;5],1);
            testCase.verifyEqual(pack,[10;12;15]);
            testCase.verifyEqual(index,[1;3;6]);
        end
        function constantCurrentStillKeepsCorrectIndices(testCase)
            [pack,~,index] = phase3battery.alignSamples((0:3).',ones(4,1),(1:3).',1);
            testCase.verifyEqual(pack,ones(3,1));
            testCase.verifyEqual(index,[2;3;4]);
        end
        function emptySubsetRetained(testCase)
            [pack,cellCurrent,index] = phase3battery.alignSamples((0:3).',ones(4,1),[],1);
            testCase.verifyEmpty(pack); testCase.verifyEmpty(cellCurrent); testCase.verifyEmpty(index);
        end
        function shortFinalIntervalNotSnapped(testCase)
            [pack,~,index] = phase3battery.alignSamples([0;.1;.13],[2;3;4],[.1;.13],1);
            testCase.verifyEqual(pack,[3;4]); testCase.verifyEqual(index,[2;3]);
        end
        function allRowsRetainedWhenNoRemoval(testCase)
            [pack,~,index] = phase3battery.alignSamples([0;1],[2;3],[0;1],1);
            testCase.verifyEqual(pack,[2;3]); testCase.verifyEqual(index,[1;2]);
        end
        function duplicatesRejected(testCase)
            testCase.verifyError(@()phase3battery.alignSamples([0;0],[1;2],0,1),'phase3battery:Order');
        end
        function reorderedOutputRejected(testCase)
            testCase.verifyError(@()phase3battery.alignSamples([0;1],[1;2],[1;0],1),'phase3battery:Order');
        end
        function mismatchedSamplesRejected(testCase)
            testCase.verifyError(@()phase3battery.alignSamples([0;1],1,1,1),'phase3battery:Length');
        end
        function almostEqualTimeNotMatched(testCase)
            testCase.verifyError(@()phase3battery.alignSamples([0;.3],[1;2],.3+eps(.3),1),'phase3battery:Subset');
        end
        function unorderedHistoryRejected(testCase)
            testCase.verifyError(@()phase3battery.evaluateAligned([0;0],[1;2],testCase.Config),'phase3battery:Order');
        end
        function missingCurrentRejected(testCase)
            testCase.verifyError(@()phase3battery.evaluateAligned([0;1],1,testCase.Config),'phase3battery:Length');
        end
    end
end
