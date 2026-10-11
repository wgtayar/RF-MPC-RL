classdef testRepositoryDisturbance < matlab.unittest.TestCase
    methods (Test)
        function rejectsNumericFlag(testCase)
            testCase.verifyError(@() simulate_mpc_horizon(struct(),struct(),struct(), ...
                struct('enable_repository_disturbance',1)), 'simulate_mpc_horizon:DisturbanceOption');
        end
        function rejectsVectorFlag(testCase)
            testCase.verifyError(@() simulate_mpc_horizon(struct(),struct(),struct(), ...
                struct('enable_repository_disturbance',[true false])), 'simulate_mpc_horizon:DisturbanceOption');
        end
        function rejectsTextFlag(testCase)
            testCase.verifyError(@() simulate_mpc_horizon(struct(),struct(),struct(), ...
                struct('enable_repository_disturbance','false')), 'simulate_mpc_horizon:DisturbanceOption');
        end
        function firstPulseMidpoint(testCase)
            [force,point] = fcn_get_disturbance(0.9,get_params(0));
            testCase.verifyEqual(force,[0;6.25;0],AbsTol=1e-12);
            testCase.verifyEqual(point,[0.301/2;0.088/2;0.05]);
        end
        function secondPulseMidpoint(testCase)
            force = fcn_get_disturbance(2.7,get_params(0));
            testCase.verifyEqual(force,[0;17.1875;0],AbsTol=1e-12);
        end
        function outsideProfile(testCase)
            testCase.verifyEqual(fcn_get_disturbance(4,get_params(0)),zeros(3,1));
        end
        function firstOnset(testCase)
            testCase.verifyEqual(fcn_get_disturbance(0.5,get_params(0)),zeros(3,1));
        end
        function secondEnd(testCase)
            testCase.verifyEqual(fcn_get_disturbance(3.1,get_params(0)),zeros(3,1),AbsTol=1e-12);
        end
    end
end
