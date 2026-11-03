classdef testPackSizingAnalysis < matlab.unittest.TestCase
    methods(Test)
        function timeWeightedQuantileRetainsObservedThreshold(tc)
            tc.verifyEqual(robotenergy.weightedQuantile([0;100;2],[.94;.01;.05],.95),2);
            tc.verifyEqual(robotenergy.weightedQuantile(1:20,ones(1,20),.95),19);
            tc.verifyEqual(robotenergy.weightedQuantile([-100;1;2],[0;1;1],0),1);
            tc.verifyEqual(robotenergy.weightedQuantile([0;100;2],[.94;.01;.05],1),100);
            tc.verifyError(@()robotenergy.weightedQuantile([1;NaN],[1;1],.95),'robotenergy:QuantileInput');
        end
        function candidateReplayConservesEnergyAndCharge(tc)
            source=bootstrap_RF_MPC_RL(); %#ok<NASGU>
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            inputRoot=fullfile(fixture.Folder,'input'); mkdir(inputRoot);
            outputRoot=fullfile(fixture.Folder,'output');
            b=setup_RL_MPC(); b.cfg.BATTERY=phase3battery.manuscriptConfig();
            save(fullfile(inputRoot,'configuration.mat'),'b');
            drive=zeros(3,53); drive(:,1)=[0;.01;.03]; drive(:,2)=[.01;.03;.06];
            drive(:,3)=[12;24;36]; drive(:,4)=1; drive(:,5)=.1;
            save(fullfile(inputRoot,'demand.mat'),'drive');
            report=run_pack_sizing_analysis(inputRoot,outputRoot);
            tc.verifyEqual(report.candidate_count,36);
            tc.verifyFalse(report.pack_frozen); tc.verifyFalse(report.training_ready);
            tc.verifyFalse(report.instantaneous_voltage_qualified);
            expectedWh=sum(drive(:,3).*(drive(:,2)-drive(:,1)))/3600;
            for k=1:numel(report.candidates)
                c=report.candidates{k}; tc.verifyTrue(c.completed_demand);
                tc.verifyEqual(c.energy_Wh,expectedWh,'AbsTol',1e-14);
                saved=load(fullfile(outputRoot,c.trace_file),'s','b','trace');
                tc.verifyEqual(saved.s.soc,c.initial_soc-c.charge_As/(3600*c.n_parallel*b.cfg.BATTERY.C_nom_Ah),'AbsTol',1e-14);
                tc.verifyEqual(c.cell_current_p95_A,max(saved.trace(:,2)));
                tc.verifyEqual(sha256_file(fullfile(outputRoot,c.trace_file)),c.trace_sha256);
            end
            tc.verifyError(@()run_pack_sizing_analysis(inputRoot,outputRoot),'packSizing:Exists');
        end
        function impossibleVoltageBoundIsNotReplayedOrRanked(tc)
            bootstrap_RF_MPC_RL();
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            inputRoot=fullfile(fixture.Folder,'input'); mkdir(inputRoot);
            b=setup_RL_MPC(); b.cfg.BATTERY=phase3battery.manuscriptConfig();
            save(fullfile(inputRoot,'configuration.mat'),'b');
            drive=zeros(1,53); drive(2)=.01; drive(3)=12; drive(4)=100; drive(5)=.1;
            save(fullfile(inputRoot,'demand.mat'),'drive');
            report=run_pack_sizing_analysis(inputRoot,fullfile(fixture.Folder,'output'));
            for k=1:numel(report.candidates)
                c=report.candidates{k}; tc.verifyFalse(c.replayed);
                tc.verifyEqual(c.status,'not_replayed_necessary_voltage_bound');
                tc.verifyTrue(isnan(c.cell_current_p95_A)); tc.verifyFalse(c.mean_model_qualified);
            end
            tc.verifyEmpty(report.both_scenarios_p95_supported_min_cell_designs);
        end
    end
end
