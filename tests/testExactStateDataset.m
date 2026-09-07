classdef testExactStateDataset < matlab.unittest.TestCase
    properties
        Folder
        Metadata
    end
    methods (TestMethodSetup)
        function temporaryDataset(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = fullfile(fixture.Folder,'dataset');
            source = fileparts(fileparts(mfilename('fullpath')));
            testCase.Metadata = struct('run_id','synthetic_unit_test', ...
                'run_type','unit_test','monitor_root',source,'seed',1, ...
                'observation_schema_version','not_applicable_synthetic', ...
                'reward_version','not_applicable_synthetic', ...
                'solver_strategy','synthetic','source_policy','synthetic');
        end
    end
    methods (Test)
        function roundTripHasLinkedExactQP(testCase)
            writer = ExactStateDataset(testCase.Folder,struct('test',true),testCase.Metadata);
            [row,problem] = testCase.syntheticRow();
            writer.beginSegment(row.state_before, ...
                struct('episode',1,'decision',1,'chunk',1),struct(), ...
                struct('action_execution',row.action_execution));
            writer.append(row,problem);
            writer.endSegment(row.state_after,struct('terminal_reason','unit_test'));
            writer.finish('unit_test_complete');
            report = validate_exact_state_dataset(testCase.Folder);
            testCase.verifyTrue(report.valid);
            testCase.verifyEqual(report.rows,1);
            testCase.verifyEqual(report.full_qps,1);
            testCase.verifyFalse(report.restore_equivalence_tested);
            testCase.verifyError(@() writer.append(row,problem),'ExactStateDataset:Closed');
        end
        function refusesOverwrite(testCase)
            ExactStateDataset(testCase.Folder,struct(),testCase.Metadata);
            testCase.verifyError(@() ExactStateDataset(testCase.Folder,struct(), ...
                testCase.Metadata),'ExactStateDataset:Exists');
        end
        function crossesBufferBoundaryWithoutLosingRows(testCase)
            writer = ExactStateDataset(testCase.Folder,struct(),testCase.Metadata);
            testCase.appendSyntheticRows(writer,257);
            writer.finish('synthetic_buffer_test_complete');
            report = validate_exact_state_dataset(testCase.Folder);
            index = readtable(fullfile(testCase.Folder,'mpc_steps','index.csv'));
            testCase.verifyEqual(report.rows,257);
            testCase.verifyEqual(report.full_qps,3);
            testCase.verifyEqual(index.row_count,[256;1]);
        end
        function detectsModifiedArtifact(testCase)
            writer = ExactStateDataset(testCase.Folder,struct(),testCase.Metadata);
            testCase.appendSyntheticRows(writer,1);
            writer.finish('synthetic_checksum_test_complete');
            tampered = true;
            save(fullfile(testCase.Folder,'manifest','configuration.mat'),'tampered');
            testCase.verifyError(@() validate_exact_state_dataset(testCase.Folder), ...
                'validate_exact_state_dataset:Checksum');
        end
        function rejectsMissingHiddenState(testCase)
            [row,~] = testCase.syntheticRow();
            row = testCase.addLinkFields(row);
            row.state_before = rmfield(row.state_before,'fsm_internal_state');
            testCase.verifyError(@() validate_exact_state_row(row,[]), ...
                'validate_exact_state_row:IncompleteState');
        end
        function rejectsDiscontinuousState(testCase)
            [row,~] = testCase.syntheticRow();
            previous = testCase.addLinkFields(row);
            row = previous;
            row.row_id = 2;
            row.time_before = 0.01;
            row.time_after = 0.02;
            row.state_before.t = 0.01;
            row.state_after.t = 0.02;
            row.state_before.Xt(1) = 7;
            testCase.verifyError(@() validate_exact_state_row(row,previous), ...
                'validate_exact_state_row:StateGap');
        end
    end
    methods (Static, Access = private)
        function appendSyntheticRows(writer,count)
            [row,problem] = testExactStateDataset.syntheticRow();
            writer.beginSegment(row.state_before, ...
                struct('episode',1,'decision',1,'chunk',1),struct(), ...
                struct('action_execution',row.action_execution));
            for k = 1:count
                row.iteration = k;
                row.time_before = (k-1)*0.01;
                row.time_after = k*0.01;
                row.state_before.t = row.time_before;
                row.state_after.t = row.time_after;
                writer.append(row,problem);
            end
            writer.endSegment(row.state_after,struct('terminal_reason','synthetic'));
        end
        function row = addLinkFields(row)
            row.configuration_sha256 = 'synthetic';
            row.source_sha = 'synthetic';
            row.row_id = 1;
            row.segment = 1;
            row.context = struct('episode',1,'decision',1,'chunk',1);
            row.snapshot_reference = struct('file','','sha256','');
        end
        function [row,problem] = syntheticRow()
            s = struct('Xt',zeros(30,1),'Ut',zeros(12,1),'t',0, ...
                'fsm_internal_state',struct('synthetic_fixture',true),'knee_proxy_state',struct(), ...
                'battery',struct(),'current_sample_count',0,'mpc_warm_start',[]);
            after = s;
            after.t = 0.01;
            cfg = struct('DR_MAX',0.0005,'DGAMMA_V_MAX',0.03, ...
                'GAMMA_V_MIN',0,'GAMMA_V_MAX',0.45,'GAMMA_A_MIN',0.1, ...
                'GAMMA_A_MAX',0.5,'V_MIN',0.3,'V_MAX',1.1,'A_MIN',0.2, ...
                'A_MAX',1,'TACC_MIN',1,'TACC_MAX',2);
            history = struct('last_R',ones(3,1),'v_req',0.8,'a_req',1, ...
                'v_exec',0.532,'prev_gamma_v',0.29,'prev_gamma_a',0.1, ...
                'previous_applied_action',[0;0;0;0.29;0.1]);
            action = resolve_supervisory_action([0;0;0;0.3;0.1],history,cfg, ...
                0.95*ones(3,1),1.05*ones(3,1));
            solver = struct('strategy','synthetic','success',true,'exitflag',1, ...
                'output',struct(),'lambda',struct(),'solver_options',[], ...
                'initial_point',[],'iterations',0,'wall_time_s',0,'diagnostics',struct());
            row = struct('time_before',0,'time_after',0.01,'iteration',1, ...
                'state_before',s,'state_after',after,'Xt_qp',s.Xt,'Ut_qp',s.Ut, ...
                'Xd',zeros(30,1),'Ud',zeros(12,1),'FSM',ones(4,1), ...
                'fsm_after',struct(),'solver',solver, ...
                'health_before',struct(),'health_after',struct(), ...
                'dynamic_event',false,'integrated',true,'current_sample_A',NaN, ...
                'current_sample_time',0,'current_sample_committed',false, ...
                'action_execution',action);
            problem = struct('H',eye(2),'g',[0;0],'Aineq',[],'bineq',[], ...
                'Aeq',[],'beq',[]);
        end
    end
end
