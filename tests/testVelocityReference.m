classdef testVelocityReference < matlab.unittest.TestCase
    methods (Test)
        function startupMatchesIntegratedConstantAcceleration(tc)
            reference = update_velocity_reference(struct('t',0),localControl(.8,.4));
            [x,v,a] = evaluate_velocity_reference(reference,[0,1,2,3]);
            tc.verifyEqual(x,[0,.2,.8,1.6],'AbsTol',1e-14);
            tc.verifyEqual(v,[0,.4,.8,.8],'AbsTol',1e-14);
            tc.verifyEqual(a,[.4,.4,0,0],'AbsTol',1e-14);
        end
        function laterSpeedIncreasePreservesDesiredPositionAndVelocity(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.6,.3));
            state = struct('t',5,'Xt',999*ones(30,1),'reference_state',previous);
            reference = update_velocity_reference(state,localControl(.9,.15));
            [x,v] = evaluate_velocity_reference(reference,[5,6,7,8]);
            tc.verifyEqual(x,[2.4,3.075,3.9,4.8],'AbsTol',1e-13);
            tc.verifyEqual(v,[.6,.75,.9,.9],'AbsTol',1e-14);
        end
        function decelerationUsesNegativeSignedAccelerationWithoutOvershoot(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.9,.3));
            reference = update_velocity_reference(struct('t',5,'reference_state',previous), ...
                localControl(.3,.2));
            [x,v,a] = evaluate_velocity_reference(reference,[5,6,8,9]);
            tc.verifyEqual(x,[3.15,3.95,4.95,5.25],'AbsTol',1e-13);
            tc.verifyEqual(v,[.9,.7,.3,.3],'AbsTol',1e-14);
            tc.verifyEqual(a,[-.2,-.2,0,0],'AbsTol',1e-14);
        end
        function midRampRetargetUsesCurrentDesiredVelocity(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.8,.2));
            reference = update_velocity_reference(struct('t',1,'reference_state',previous), ...
                localControl(.1,.05));
            [x,v] = evaluate_velocity_reference(reference,[1,2,3,4]);
            tc.verifyEqual(x,[.1,.275,.4,.5],'AbsTol',1e-14);
            tc.verifyEqual(v,[.2,.15,.1,.1],'AbsTol',1e-14);
        end
        function accelerationOnlyChangeRetimesRemainingRamp(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.8,.4));
            reference = update_velocity_reference(struct('t',1,'reference_state',previous), ...
                localControl(.8,.2));
            [x,v,a] = evaluate_velocity_reference(reference,[1,2,3]);
            tc.verifyEqual(x,[.2,.7,1.4],'AbsTol',1e-14);
            tc.verifyEqual(v,[.4,.6,.8],'AbsTol',1e-14);
            tc.verifyEqual(a,[.2,.2,0],'AbsTol',1e-14);
        end
        function accelerationOnlyChangeAtCruiseDoesNotInventMotion(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.6,.3));
            reference = update_velocity_reference(struct('t',5,'reference_state',previous), ...
                localControl(.6,.8));
            [x,v,a] = evaluate_velocity_reference(reference,[5,6,7]);
            tc.verifyEqual(x,[2.4,3,3.6],'AbsTol',1e-14);
            tc.verifyEqual(v,[.6,.6,.6],'AbsTol',1e-14);
            tc.verifyEqual(a,[0,0,0]);
        end
        function repeatedCommandsAreChunkInvariant(tc)
            control = localControl(.8,.4);
            original = update_velocity_reference(struct('t',0),control);
            retained = original;
            for time = [.01,.13,.61,1.2,1.999,2,2.5,5]
                retained = update_velocity_reference(struct('t',time,'reference_state',retained),control);
                tc.verifyEqual(retained,original);
            end
            [x1,v1] = evaluate_velocity_reference(original,[5,5.01,6]);
            [x2,v2] = evaluate_velocity_reference(retained,[5,5.01,6]);
            tc.verifyEqual(x1,x2);
            tc.verifyEqual(v1,v2);
        end
        function zeroTargetStopsAndHoldsPosition(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.6,.3));
            reference = update_velocity_reference(struct('t',5,'reference_state',previous), ...
                localControl(0,.3));
            [x,v] = evaluate_velocity_reference(reference,[5,6,7,8]);
            tc.verifyEqual(x,[2.4,2.85,3,3],'AbsTol',1e-14);
            tc.verifyEqual(v,[.6,.3,0,0],'AbsTol',1e-14);
        end
        function boundaryHasContinuousPositionVelocity(tc)
            previous = update_velocity_reference(struct('t',0),localControl(.8,.3));
            for time = [.7,5]
                reference = update_velocity_reference(struct('t',time,'reference_state',previous), ...
                    localControl(.4,.2));
                [x0,v0] = evaluate_velocity_reference(previous,time);
                [x1,v1] = evaluate_velocity_reference(reference,time);
                tc.verifyEqual(x1,x0);
                tc.verifyEqual(v1,v0);
                boundary = time+abs(reference.v_cmd-reference.v_start_mps)/reference.a_cmd;
                h = 1e-6;
                [x,v] = evaluate_velocity_reference(reference,[boundary-h,boundary,boundary+h]);
                tc.verifyEqual((x(3)-x(1))/(2*h),reference.v_cmd,'AbsTol',1e-7);
                tc.verifyEqual(v(2),reference.v_cmd);
                tc.verifyLessThanOrEqual(max(abs(diff(v))),reference.a_cmd*h+1e-14);
            end
        end
        function incompatibleHistoryAndBackwardTimeFailClosed(tc)
            tc.verifyError(@() update_velocity_reference(struct('t',5),localControl(.6,.3)), ...
                'update_velocity_reference:MissingHistory');
            legacy = struct('schema_version','reference_position_state_v1','mode','position_continuous_v1', ...
                't_start_s',0,'x_start_m',0,'v_start_mps',0,'v_cmd',.6,'a_cmd',.3);
            tc.verifyError(@() update_velocity_reference(struct('t',5,'reference_state',legacy),localControl(.6,.3)), ...
                'evaluate_velocity_reference:Schema');
            reference = update_velocity_reference(struct('t',0),localControl(.6,.3));
            reference = update_velocity_reference(struct('t',5,'reference_state',reference),localControl(.8,.3));
            tc.verifyError(@() evaluate_velocity_reference(reference,4.99), ...
                'evaluate_velocity_reference:BeforeSegment');
        end
        function evaluationPreservesArrayShape(tc)
            reference = update_velocity_reference(struct('t',0),localControl(.8,.4));
            [x,v,a] = evaluate_velocity_reference(reference,[0;1;2]);
            tc.verifySize(x,[3,1]); tc.verifySize(v,[3,1]); tc.verifySize(a,[3,1]);
        end
    end
end

function control = localControl(velocity, acceleration)
    control = struct('v_cmd',velocity,'a_cmd',acceleration,'reference_mode','velocity_continuous_v2');
end
