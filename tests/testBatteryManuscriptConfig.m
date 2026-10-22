classdef testBatteryManuscriptConfig < matlab.unittest.TestCase
    properties
        Root
    end
    methods (TestMethodSetup)
        function rootFolder(testCase)
            testCase.Root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(testCase.Root));
        end
    end
    methods (Test)
        function manuscriptNoiseAndReportedErrorAreSeparate(testCase)
            cfg = phase3battery.manuscriptConfig();
            [~,noise] = phase3battery.resolveModelOptions(cfg.model_options,'unused.xlsx');
            testCase.verifyEqual(noise.process_covariance,diag([1e-5 1e-3 1e-3 1e-3]));
            testCase.verifyEqual(noise.measurement_variance,1e-3);
            testCase.verifyEqual(cfg.assumptions.soc_rmse_assumed_percentage_points, ...
                100*cfg.assumptions.soc_rmse_assumed_fraction,'AbsTol',eps);
            testCase.verifyEqual(cfg.feedback_version,'battery_feedback_timestamp_aligned_v2');
        end
        function missingOptionsKeepHistoricalNoise(testCase)
            [~,noise] = phase3battery.resolveModelOptions(struct(),fullfile(testCase.Root,'our params.xlsx'));
            testCase.verifyEqual(noise.process_covariance,diag([1e-7 1e-3 1e-3 1e-3]));
        end
        function experimentalTableIsNotChemistryCorrected(testCase)
            cfg = phase3battery.manuscriptConfig();
            params = phase3battery.resolveModelOptions(cfg.model_options,'unused.xlsx');
            original = readtable(fullfile(testCase.Root,'our params.xlsx'));
            testCase.verifyEqual(params,original);
            testCase.verifyEqual(params.OCV(end),4.1816);
            testCase.verifyEqual(cfg.assumptions.cell_nominal_voltage_reported_V,3.7);
        end
        function tableInjectionChangesBothModelAndEstimator(testCase)
            params = readtable(fullfile(testCase.Root,'our params.xlsx'));
            options = struct('parameter_source',params,'voltage_filter','none');
            times = (0:.1:40).'; current = 2+sin(times);
            [~,ta,va] = model_battery(current,times,2,.95,false,options);
            params.OCV = params.OCV+.1;
            changed = struct('parameter_source',params,'voltage_filter','none');
            [~,tb,vb] = model_battery(current,times,2,.95,false,changed);
            testCase.verifyEqual(ta,tb);
            testCase.verifyEqual(vb-va,.1*ones(size(va)),'AbsTol',1e-10);
            cellCurrent = current(ismember(times,ta));
            a = estimateSOC(ta,cellCurrent,va,2,.95,false,options);
            b = estimateSOC(tb,cellCurrent,vb,2,.95,false,changed);
            testCase.verifyEqual(a,b,'AbsTol',1e-9);
        end
        function noiseOverrideAffectsVoltageCorrection(testCase)
            cfg = phase3battery.manuscriptConfig();
            times = (0:.1:40).'; current = 2+sin(times);
            [~,tb,v] = model_battery(current,times,2,.95,false,cfg.model_options);
            current = current(ismember(times,tb));
            v = v+.03*sin(tb/3);
            legacy = estimateSOC(tb,current,v,2,.95,false);
            manuscript = estimateSOC(tb,current,v,2,.95,false,cfg.model_options);
            testCase.verifyGreaterThan(max(abs(legacy-manuscript)),1e-6);
        end
        function alignedDispatchUsesCustomTableAndNoise(testCase)
            params = readtable(fullfile(testCase.Root,'our params.xlsx'));
            cfg = phase3battery.manuscriptConfig(params);
            times = (0:.1:40).'; current = 12+3*sin(times);
            battery = evaluate_battery_feedback(times,current,cfg);
            expected = estimateSOC(battery.trace_time,battery.trace_current, ...
                battery.trace_voltage,cfg.C_nom_Ah,cfg.SOC_init,false,cfg.model_options);
            testCase.verifyEqual(battery.trace_metric,expected(:));
            testCase.verifyTrue(all(isfinite(battery.trace_metric)));
        end
        function largerParallelPackReducesCellLoad(testCase)
            cfg = phase3battery.manuscriptConfig(); cfg.decim = 1;
            times = (0:.1:40).'; current = 12+3*sin(times);
            a = evaluate_battery_feedback(times,current,cfg);
            cfg.n_parallel = 12;
            b = evaluate_battery_feedback(times,current,cfg);
            testCase.verifyEqual(b.trace_current,a.trace_current/2);
            testCase.verifyGreaterThan(b.soc_pct,a.soc_pct);
            testCase.verifyLessThan(a.soc_pct,100*cfg.SOC_init);
            testCase.verifyLessThan(b.soc_pct,100*cfg.SOC_init);
        end
        function rawVoltageAvoidsLegacyFilterEndpointBias(testCase)
            cfg = phase3battery.manuscriptConfig();
            times = (0:.1:40).'; current = 2*ones(size(times));
            [~,tb,raw] = model_battery(current,times,2,.95,false,cfg.model_options);
            legacyOptions = cfg.model_options;
            legacyOptions.voltage_filter = 'legacy_lowpass';
            [~,tl,filtered] = model_battery(current,times,2,.95,false,legacyOptions);
            testCase.verifyEqual(tb,tl);
            testCase.verifyGreaterThan(max(abs(filtered-raw)),.01);
            params = phase3battery.resolveModelOptions(cfg.model_options,'unused.xlsx');
            initialOcv = interp1(params.SOC,params.OCV,.95,'pchip');
            testCase.verifyLessThan(max(raw),initialOcv);
        end
        function invalidParametersRejected(testCase)
            params = readtable(fullfile(testCase.Root,'our params.xlsx'));
            params.R0(2) = -1;
            testCase.verifyError(@()phase3battery.resolveModelOptions( ...
                struct('parameter_source',params),'unused.xlsx'),'phase3battery:Parameters');
        end
        function unknownOptionRejected(testCase)
            testCase.verifyError(@()phase3battery.resolveModelOptions( ...
                struct('process_covarianc',eye(4)),'unused.xlsx'),'phase3battery:ModelOptions');
        end
        function invalidCovarianceRejected(testCase)
            cfg = phase3battery.manuscriptConfig();
            cfg.model_options.process_covariance = diag([-1 1 1 1]);
            testCase.verifyError(@()phase3battery.resolveModelOptions( ...
                cfg.model_options,'unused.xlsx'),'phase3battery:Covariance');
        end
    end
end
