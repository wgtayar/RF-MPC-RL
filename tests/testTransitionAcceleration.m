classdef testTransitionAcceleration < matlab.unittest.TestCase
    % Action authority and reference integration; no robot rollout or files.
    methods (TestClassSetup)
        function bootstrap(~)
            bootstrap_RF_MPC_RL();
        end
    end
    methods (Test)
        function allConfiguredSpeedsRetainDistinctAccelerationActions(tc)
            cfg = localConfig();
            speeds = [.3,320/600,.8,1,1.2];
            gammaA = [.1,.2,.3,.4,.5];
            expected = [.2,.275,.35,.425,.5];
            for speed = speeds
                gammaV = (speed-cfg.V_MIN)/(cfg.V_MAX-cfg.V_MIN);
                actual = zeros(size(gammaA));
                for k = 1:numel(gammaA)
                    [v,actual(k)] = apply_command_governor(.5333333333333333,1,gammaV,gammaA(k),cfg);
                    tc.verifyEqual(v,speed,'AbsTol',1e-14);
                end
                tc.verifyEqual(actual,expected,'AbsTol',1e-14);
                tc.verifyGreaterThan(min(diff(actual)),0);
            end
        end
        function clippedActionsReachExistingDeclaredBounds(tc)
            cfg = localConfig();
            [vLo,aLo] = apply_command_governor(.5333333333333333,1,-1,-1,cfg);
            [vHi,aHi] = apply_command_governor(.5333333333333333,1,2,2,cfg);
            tc.verifyEqual([vLo,aLo],[.3,.2],'AbsTol',1e-14);
            tc.verifyEqual([vHi,aHi],[1.2,.5],'AbsTol',1e-14);
            % Scaling a_req changes the declared endpoint-derived range.
            [~,a0] = apply_command_governor(.5333333333333333,2,.5,.1,cfg);
            [~,am] = apply_command_governor(.5333333333333333,2,.5,.3,cfg);
            [~,a1] = apply_command_governor(.5333333333333333,2,.5,.5,cfg);
            tc.verifyEqual([a0,am,a1],[.2,.6,1],'AbsTol',1e-14);
        end
        function archivedConfigurationRetainsOriginalGovernor(tc)
            cfg = rmfield(localConfig(),'ACCELERATION');
            speeds = [.3,320/600,.8,1,1.2];
            expectedMin = [.2,(320/600)/2,.4,.5,.6];
            expectedMax = [.5,.5,.5,.5,.6];
            for k = 1:numel(speeds)
                gammaV = (speeds(k)-cfg.V_MIN)/(cfg.V_MAX-cfg.V_MIN);
                [~,aLo] = apply_command_governor(.5333333333333333,1,gammaV,.1,cfg);
                [~,aHi] = apply_command_governor(.5333333333333333,1,gammaV,.5,cfg);
                tc.verifyEqual([aLo,aHi],[expectedMin(k),expectedMax(k)],'AbsTol',1e-14);
            end
        end
        function configuredReferenceModeRetainsArchivedDefault(tc)
            tc.verifyEqual(configured_reference_mode(struct()),'position_continuous_v1');
            cfg = struct('REFERENCE',struct('mode','velocity_continuous_v2'));
            tc.verifyEqual(configured_reference_mode(cfg),'velocity_continuous_v2');
            cfg.REFERENCE.mode = 'position_continuous_v1';
            tc.verifyEqual(configured_reference_mode(cfg),'position_continuous_v1');
        end
        function freshMissionDeclaresBothTransitionModels(tc)
            bundle = setup_energy_mission_rl();
            tc.verifyEqual(bundle.cfg.REFERENCE.mode,'velocity_continuous_v2');
            tc.verifyEqual(bundle.cfg.ACCELERATION.action_model,'bounded_transition_v1');
            tc.verifyEqual(configured_reference_mode(bundle.cfg),'velocity_continuous_v2');
            tc.verifyEqual([bundle.cfg.BATTERY.n_series,bundle.cfg.BATTERY.n_parallel, ...
                bundle.cfg.BATTERY.C_nom_Ah],[9,1,2]);
        end
        function startupReferenceMatchesOriginalIntegratedMotion(tc)
            p = get_params(0); p.vel_d = [.8;0]; p.acc_d = .5;
            time = [0,.5,1.6,3,7]; stance = ones(4,numel(time));
            [oldXd,oldUd] = fcn_gen_XdUd(time,[],stance,p);
            control = struct('v_cmd',.8,'a_cmd',.5,'reference_mode','velocity_continuous_v2');
            p.velocity_reference = update_velocity_reference(struct('t',0),control);
            [newXd,newUd] = fcn_gen_XdUd(time,[],stance,p);
            tc.verifyEqual(newXd,oldXd,'AbsTol',1e-13);
            tc.verifyEqual(newUd,oldUd);
            tc.verifyEqual(newXd(1,:),[0,.0625,.64,1.76,4.96],'AbsTol',1e-13);
            tc.verifyEqual(newXd(4,:),[0,.25,.8,.8,.8],'AbsTol',1e-14);
        end
        function fsmInputGeneratorUsesLaterRampInsteadOfAbsoluteTime(tc)
            p = get_params(0); p.vel_d = [.5573333333333333;0]; p.acc_d = .2;
            old = update_velocity_reference(struct('t',0), ...
                struct('v_cmd',320/600,'a_cmd',.5,'reference_mode','velocity_continuous_v2'));
            p.velocity_reference = update_velocity_reference(struct('t',5,'reference_state',old), ...
                struct('v_cmd',320/600+.024,'a_cmd',.2,'reference_mode','velocity_continuous_v2'));
            time = [5,5.06,5.12,5.3];
            [xd,~] = fcn_gen_XdUd(time,[],ones(4,numel(time)),p);
            tc.verifyEqual(xd(4,:),[320/600,320/600+.012,320/600+.024,320/600+.024], ...
                'AbsTol',1e-13);
            tc.verifyEqual(xd(2,:),zeros(size(time)));
            tc.verifyEqual(xd(5,:),zeros(size(time)));
            % A flat target would expose the old late-command defect.
            tc.verifyLessThan(xd(4,1),p.vel_d(1));
        end
    end
end

function cfg = localConfig()
cfg = struct('V_MIN',.3,'V_MAX',1.2,'GAMMA_V_MIN',0,'GAMMA_V_MAX',1, ...
    'GAMMA_A_MIN',.1,'GAMMA_A_MAX',.5,'A_MIN',.2,'A_MAX',4, ...
    'TACC_MIN',.2,'TACC_MAX',2, ...
    'ACCELERATION',struct('action_model','bounded_transition_v1'));
end
