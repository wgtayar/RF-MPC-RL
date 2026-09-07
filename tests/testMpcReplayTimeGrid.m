classdef testMpcReplayTimeGrid < matlab.unittest.TestCase
    methods (Test)
        function legacyGridIsUnchanged(testCase)
            [before,after] = mpc_replay_time_grid(52,0.01,300,struct());
            testCase.verifyEqual(before,52+0.01*(0:299));
            testCase.verifyEqual(after,52+0.01*(1:300));
        end
        function interiorGridMatchesOriginalArithmetic(testCase)
            [before,after] = mpc_replay_time_grid(52,0.01,300, ...
                struct('integration_time_origin_s',50));
            testCase.verifyEqual(before,50+0.01*(200:499));
            testCase.verifyEqual(after,50+0.01*(201:500));
            testCase.verifyNotEqual(after(223),52+0.01*223);
        end
        function boundaryGridMatchesOriginal(testCase)
            [before,after] = mpc_replay_time_grid(50,0.01,500, ...
                struct('integration_time_origin_s',50));
            testCase.verifyEqual(before,50+0.01*(0:499));
            testCase.verifyEqual(after,50+0.01*(1:500));
        end
        function offGridRestoreIsRejected(testCase)
            testCase.verifyError(@() mpc_replay_time_grid(52.005,0.01,300, ...
                struct('integration_time_origin_s',50)), 'mpc_replay_time_grid:OffGrid');
        end
    end
end
