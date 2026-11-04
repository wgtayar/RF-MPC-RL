classdef testPackSizingBound < matlab.unittest.TestCase
    methods(Test)
        function plantConservesChargeAndEnergyWithoutEstimatorUpdates(tc)
            bootstrap_RF_MPC_RL(); b=phase3battery.manuscriptConfig();
            s=robotenergy.start(b); initial=s; power=[0 12 80 25 100]; dt=[.001 .01 .003 .02 .001];
            expectedWh=0;
            for k=1:numel(power)
                [s,y]=robotenergy.advancePlant(s,power(k),dt(k));
                expectedWh=expectedWh+power(k)*dt(k)/3600;
                tc.verifyEqual(y.packA*y.packV,power(k),'AbsTol',1e-11);
            end
            tc.verifyEqual(s.estimate,initial.estimate); tc.verifyEqual(s.cov,initial.cov);
            tc.verifyEqual(s.energyWh,expectedWh,'AbsTol',1e-14);
            tc.verifyEqual(s.soc,initial.soc-s.chargeAs/(3600*s.np*s.capacity),'AbsTol',1e-14);
            tc.verifyEqual(s.elapsed,sum(dt),'AbsTol',1e-14);
        end
        function sufficientBoundSurvivesWorstSupportedPower(tc)
            bootstrap_RF_MPC_RL(); b=phase3battery.manuscriptConfig();
            bound=robotenergy.conservativePackBound(b,60,400,20,.25);
            tc.verifyGreaterThan(bound.high_root_monotonicity_margin_V,0);
            tc.verifyGreaterThanOrEqual(bound.guaranteed_loaded_bus_lower_bound_V,60);
            tc.verifyGreaterThanOrEqual(bound.n_series*bound.n_parallel*bound.cell_voltage_lower_bound_V* ...
                b.C_nom_Ah*(.25-b.terminal_margin),20);
            s=robotenergy.start(b);
            for soc=linspace(.2,1,101)
                s.soc=soc; [r,ocv]=robotenergy.lookup(s,soc);
                tc.verifyGreaterThanOrEqual(ocv,bound.ocv_min_V);
                tc.verifyLessThanOrEqual(ocv,bound.ocv_max_V);
                tc.verifyLessThanOrEqual(r(1:4)',bound.maximum_resistances_ohm+1e-14);
                s.rc=bound.maximum_resistances_ohm(2:4)'*bound.characterization_current_A;
                P=s.ns*s.np*bound.cell_voltage_lower_bound_V*bound.characterization_current_A;
                instant=robotenergy.instantaneousPack(s,P);
                [next,y]=robotenergy.advancePlant(s,P,.001);
                tc.verifyLessThanOrEqual(max(instant.cellA,y.cellA),bound.characterization_current_A+1e-12);
                tc.verifyGreaterThanOrEqual(min(instant.cellV,y.cellV),bound.cell_voltage_lower_bound_V-1e-12);
                tc.verifyLessThanOrEqual(next.rc,s.rc+1e-12);
            end
        end
        function zeroDurationCheckHasNoEnergyOrCharge(tc)
            bootstrap_RF_MPC_RL(); b=phase3battery.manuscriptConfig(); s=robotenergy.start(b);
            saved=s; y=robotenergy.instantaneousPack(s,50);
            tc.verifyEqual(s,saved); tc.verifyEqual(y.packV*y.packA,50,'AbsTol',1e-12);
            tc.verifyGreaterThan(y.cellA,0); tc.verifyEqual(s.energyWh,0); tc.verifyEqual(s.chargeAs,0);
            tc.verifyError(@()robotenergy.instantaneousPack(s,1e9),'robotenergy:PowerLimit');
        end
        function syntheticFullTraceRetainsEndpointsAndConservation(tc)
            bootstrap_RF_MPC_RL(); fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            inputRoot=fullfile(fixture.Folder,'input'); mkdir(inputRoot); outputRoot=fullfile(fixture.Folder,'output');
            b=setup_RL_MPC('legacy_frozen_v1'); b.cfg.BATTERY=phase3battery.manuscriptConfig();
            save(fullfile(inputRoot,'configuration.mat'),'b');
            steps=4; fine=4; dt=.01/fine; N=steps*fine;
            sample_time_s=((1:N)'-.5)*dt; watts=(1:N)'+10; required_voltage_V=ones(N,1);
            motor_torque_peak_Nm=.1*ones(N,1); comparison_watts=20*ones(N/2,1); endpoint_watts=400*ones(steps,2);
            endpoint_required_voltage_V=60*ones(steps,2); endpoint_motor_torque_peak_Nm=.2*ones(steps,2);
            step_summary=zeros(steps,8); step_summary(:,4)=60; step_summary(:,5)=400; step_summary(:,6)=.2;
            save(fullfile(inputRoot,'instantaneous.mat'),'sample_time_s','watts','required_voltage_V', ...
                'motor_torque_peak_Nm','comparison_watts','endpoint_watts','endpoint_required_voltage_V', ...
                'endpoint_motor_torque_peak_Nm','step_summary','-v7.3');
            manifest=struct('options',struct('substeps',fine,'comparison_substeps',fine/2),'midpoint_duration_s',dt, ...
                'input_configuration_sha256',sha256_file(fullfile(inputRoot,'configuration.mat')));
            result=struct('full_trace_analyzed',true,'sample_file','instantaneous.mat', ...
                'sample_file_sha256',sha256_file(fullfile(inputRoot,'instantaneous.mat')), ...
                'processed_steps',steps,'input_steps',steps,'duration_s',steps*.01,'source_sha','synthetic-test-fixture','fine',struct('samples',N), ...
                'energy_relative_grid_difference',0);
            localJson(fullfile(inputRoot,'manifest.json'),manifest); localJson(fullfile(inputRoot,'result.json'),result);
            report=run_pack_sizing_freeze(inputRoot,outputRoot);
            tc.verifyTrue(report.fixed_simulation_specification_selected); tc.verifyFalse(report.training_ready);
            tc.verifyEqual(report.all_amplitude_reference_bound.sampled_peak_voltage_V,60); tc.verifyEqual(report.all_amplitude_reference_bound.sampled_peak_power_W,400);
            tc.verifyEqual(report.all_amplitude_reference_bound.mission_energy_Wh,sum(watts)*dt/3600,'AbsTol',1e-15);
            for k=1:numel(report.selected.cases)
                c=report.selected.cases{k}; tc.verifyTrue(c.sampled_electrical_qualified);
                tc.verifyEqual(c.energy_Wh,sum(watts)*dt/3600,'AbsTol',1e-14);
                tc.verifyEqual(c.estimator_updates,0); tc.verifyGreaterThan(c.minimum_true_soc,.2);
                tc.verifyGreaterThanOrEqual(c.minimum_conservative_interval_voltage_headroom_V,0);
                tc.verifyEqual(sha256_file(fullfile(outputRoot,c.file)),c.file_sha256);
            end
        end
    end
end
function localJson(path,value)
fid=fopen(path,'w'); cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(value));
end
