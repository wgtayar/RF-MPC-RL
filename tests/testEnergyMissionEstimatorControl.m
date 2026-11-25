classdef testEnergyMissionEstimatorControl < matlab.unittest.TestCase
    % A4 control: an EKF whose R0..R3 are re-identified to the plant
    % (estimator_resistance_scale). Diagnosis of 25 Nov (monitor #49): with
    % the nominal EKF, a 1.3x plant resistance and a pulsed (gait-like) load,
    % the SOC estimate locks onto peaks of the non-monotone R0(SOC) table.
    % The control must (1) leave every default path unchanged, (2) track the
    % true SOC under the same mismatch, and (3) be declared and recorded.
    properties
        Folder
        Source
        Options
    end
    methods(TestClassSetup)
        function prepare(tc)
            tc.Source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(tc.Source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)),'Evaluation needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Options=struct('policy','fixed','speed_m_s',1.0,'start_soc',.95,'min_start_soc',.9495, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
        end
    end
    methods(Test)
        function estimatorViewIsIdentityByDefault(tc)
            s=robotenergy.start(tc.battery(1,[]));
            tc.verifyEqual(robotenergy.estimatorView(s),s,'No mismatch: the estimator sees the state itself.');
            s=robotenergy.start(tc.battery(1.3,[]));
            tc.verifyEqual(robotenergy.estimatorView(s).values,s.values,'A plant mismatch alone keeps the nominal EKF.');
            s=robotenergy.start(tc.battery(1.3,1.3));
            q=robotenergy.estimatorView(s);
            tc.verifyEqual(q.values(:,1:4),1.3*s.values(:,1:4),'RelTol',1e-15);
            tc.verifyEqual(q.values(:,5:7),s.values(:,5:7),'Capacitances stay nominal.');
            tc.verifyEqual(q.capacity,s.capacity,'Estimator capacity stays nominal.');
        end

        function mismatchDeclarationIsValidated(tc)
            b=tc.battery(1.3,1.3);
            p=robotenergy.plantMismatch(b);
            tc.verifyEqual(p.estimator_resistance_scale,1.3);
            b.plant_mismatch.unknown=1;
            tc.verifyError(@()robotenergy.plantMismatch(b),'robotenergy:PlantMismatch');
            b=tc.battery(1.3,-1);
            tc.verifyError(@()robotenergy.plantMismatch(b),'MATLAB:expectedPositive');
        end

        function reidentifiedEstimatorTracksUnderPulsedLoad(tc)
            % 60 s of a gait-like pulsed load (57 W mean, +/-90 % square wave
            % at 0.45 s) from 0.90 SOC, with the plant resistance x1.3.
            [nominalGap,~]=tc.replay(1.3,[],60);
            [matchedGap,truth]=tc.replay(1.3,1.3,60);
            tc.verifyLessThan(abs(matchedGap),1e-6,'The re-identified EKF tracks the true SOC.');
            tc.verifyGreaterThan(abs(nominalGap),1e-3,'The nominal EKF drifts under the same mismatch.');
            tc.verifyLessThan(truth,.90,'The battery discharged.');
        end

        function shiftEvaluationDeclaresAndRecordsTheControl(tc)
            r=tc.shift('control',struct('plant_resistance_scale',1.3,'estimator_resistance_scale',1.3));
            m=r.missions(1);
            tc.verifyTrue(m.mission_complete);tc.verifyEqual(r.failed_qps,0);tc.verifyTrue(m.dataset_validated);
            tc.verifyEqual([m.plant_resistance_scale,m.estimator_resistance_scale],[1.3 1.3]);
            tc.verifyEqual(r.plant.estimator_resistance_scale,1.3);
            dataset=fullfile(tc.Folder,'control','mission_01');
            saved=load(fullfile(dataset,'manifest','configuration.mat'),'cfg');
            tc.verifyEqual(saved.cfg.BATTERY.plant_mismatch.estimator_resistance_scale,1.3);
            report=validate_exact_state_dataset(dataset,true);
            tc.verifyTrue(report.plant_mismatch.declared);
            % Without the control option the declaration carries no estimator field.
            n=tc.shift('r1',struct('plant_resistance_scale',1.3));
            tc.verifyEqual(n.missions(1).estimator_resistance_scale,1);
            saved=load(fullfile(tc.Folder,'r1','mission_01','manifest','configuration.mat'),'cfg');
            tc.verifyFalse(isfield(saved.cfg.BATTERY.plant_mismatch,'estimator_resistance_scale'));
            % The control requires a declared plant mismatch.
            bad=tc.Options;bad.estimator_resistance_scale=1.3;
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'),bad),'shiftEval:Plant');
        end
    end
    methods
        function b=battery(~,rscale,escale)
            b=phase3battery.fixedPackConfig(energy_mission_pack_file());
            b.SOC_init=.90;
            if rscale~=1 || ~isempty(escale)
                b.plant_mismatch=struct('version','plant_battery_mismatch_v1','resistance_scale',rscale, ...
                    'capacity_Ah',b.C_nom_Ah,'soc_estimate_offset',0,'scope','test');
                if ~isempty(escale),b.plant_mismatch.estimator_resistance_scale=escale;end
            end
        end

        function [gap,truth]=replay(tc,rscale,escale,seconds)
            % Mirrors robotenergy.sampleResolved at constant power per 10 ms step.
            s=robotenergy.start(tc.battery(rscale,escale));
            dt=.01;n=4;h=dt/(2*n);
            for t=1:round(seconds/dt)
                P=57*(1+.9*sign(sin(2*pi*(t-.5)*dt/.45)));
                z=s.estimate;Ftotal=eye(4);priorCov=s.cov;
                for k=1:2*n
                    [s,a]=robotenergy.advancePlant(s,P,h);
                    [z,~,F]=robotenergy.linearizeEstimator(s,z,a.cellA,h);Ftotal=F*Ftotal;
                end
                last=robotenergy.instantaneousPack(s,P);
                [r,ocv,dr,docv]=robotenergy.lookup(robotenergy.estimatorView(s),z(1));
                predictedV=ocv-r(1)*last.cellA-sum(z(2:4));H=[docv-dr(1)*last.cellA,-1,-1,-1];
                C=Ftotal*priorCov*Ftotal'+s.Q;K=C*H'/(H*C*H'+s.Rv);
                z=z+K*(last.cellV-predictedV);z(1)=max(s.grid(1),min(s.grid(end),z(1)));s.estimate=z;
                M=eye(4)-K*H;s.cov=M*C*M'+K*s.Rv*K';
            end
            gap=s.estimate(1)-s.soc;truth=s.soc;
        end

        function r=shift(tc,name,extra)
            o=tc.Options;
            for n=fieldnames(extra).',o.(n{1})=extra.(n{1});end
            root=fullfile(tc.Folder,name);
            evalc('r=run_energy_mission_shift_evaluation(root,o);');
        end
    end
end
