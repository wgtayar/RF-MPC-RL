classdef testFixedPackFailureHandling < matlab.unittest.TestCase
    % Fault injection verifies bookkeeping; it is not an operating experiment.
    properties
        Bundle
        Folder
        Metadata
    end
    methods (TestMethodSetup)
        function configure(tc)
            src=bootstrap_RF_MPC_RL();
            tc.Bundle=setup_RL_MPC();
            tc.Bundle.cfg.CHUNK_DURATION=.03;
            tc.Bundle.cfg.APPLY_EVERY=1;
            tc.Bundle.cfg.MISSION_DURATION=1;
            tc.Bundle.cfg.MISSION.D_TARGET_M=1;
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fullfile(fixture.Folder,'capture');
            tc.Metadata=struct('run_id','fixed_pack_failure_unit','run_type','unit_test', ...
                'monitor_root',fullfile(fileparts(src),'RL-MPC-Monitor'),'seed',1, ...
                'source_policy','one_step_fault_injection_not_operating_evidence');
        end
    end
    methods (Test)
        function powerLimitIsCapturedAndReturnsFiniteTerminal(tc)
            b=tc.Bundle; b.cfg.ENERGY.auxW=1e8;
            env=Phase3MpcEnvironment(b,tc.Folder,tc.Metadata);
            reset(env); before=env.State;
            [obs,reward,done,info]=step(env,localAction(b));
            tc.verifyTrue(done);
            tc.verifyEqual(info.terminal_reason,'battery_power_limit');
            tc.verifyTrue(all(isfinite(obs)) && isfinite(reward));
            tc.verifyLessThan(reward,0);
            tc.verifyEqual(info.window.charge_As,0);
            tc.verifyEqual(info.window.duration_s,0);
            tc.verifyEqual(env.State.energy,before.energy);
            close(env,'unit_test_complete');
            report=validate_exact_state_dataset(tc.Folder,true);
            tc.verifyTrue(report.valid);
            tc.verifyEqual(report.rows,1);
            tc.verifyEqual(report.windows_reconstructed,1);
            row=localFirstRow(tc.Folder);
            localVerifyUncommitted(tc,row,'robotenergy:PowerLimit');
            tc.verifyGreaterThan(row.solver.energy_exception.load.watts,1e8);
        end
        function emptyPackIsCapturedWithoutInventedEnergy(tc)
            b=tc.Bundle;
            initial=robotenergy.start(b.cfg.BATTERY);
            b.cfg.BATTERY.SOC_init=initial.grid(1);
            b.cfg.BATTERY.metric_init=100*initial.grid(1);
            env=Phase3MpcEnvironment(b,tc.Folder,tc.Metadata);
            reset(env); before=env.State;
            [obs,reward,done,info]=step(env,localAction(b));
            tc.verifyTrue(done);
            tc.verifyEqual(info.terminal_reason,'battery_depleted');
            tc.verifyTrue(all(isfinite(obs)) && isfinite(reward));
            tc.verifyEqual(env.State.energy,before.energy);
            close(env,'unit_test_complete');
            tc.verifyTrue(validate_exact_state_dataset(tc.Folder,true).valid);
            localVerifyUncommitted(tc,localFirstRow(tc.Folder),'robotenergy:Empty');
        end
        function candidateRewardTreatsPowerLimitAsFailure(tc)
            b=tc.Bundle; b.cfg.ENERGY.auxW=1e8;
            options=struct('reward_schema','reward_phase3_coverage_candidate_v1', ...
                'reward_policy',phase3reward.unitPolicy('unit_reference_experimental_v1'));
            env=Phase3MpcEnvironment(b,tc.Folder,tc.Metadata,options);
            reset(env);
            [obs,reward,done,info]=step(env,localAction(b));
            tc.verifyTrue(done);
            tc.verifyTrue(all(isfinite(obs)) && isfinite(reward));
            tc.verifyLessThan(reward,0);
            tc.verifyEqual(string(info.reward_info.actual_terminal_reason),"battery_power_limit");
            tc.verifyLessThanOrEqual(info.reward_info.pace_contribution,0);
            close(env,'unit_test_complete');
            report=validate_exact_state_dataset(tc.Folder,true);
            tc.verifyTrue(report.valid);
            tc.verifyEqual(report.windows_reconstructed,1);
            tc.verifyEqual(report.rewards_reconstructed,1);
        end
        function unexpectedErrorRethrowsWithAttemptEvidence(tc)
            b=tc.Bundle; b.cfg.ENERGY=rmfield(b.cfg.ENERGY,'auxW');
            env=Phase3MpcEnvironment(b,tc.Folder,tc.Metadata);
            reset(env);
            tc.verifyError(@()step(env,localAction(b)),'MATLAB:nonExistentField');
            tc.verifyTrue(env.Writer.Closed);
            report=validate_exact_state_dataset(tc.Folder);
            tc.verifyTrue(report.valid);
            tc.verifyEqual(report.run_status,'environment_exception');
            row=localFirstRow(tc.Folder);
            localVerifyUncommitted(tc,row,'MATLAB:nonExistentField');
            tc.verifyEmpty(row.solver.energy_exception.terminal_reason);
        end
        function classificationIsExplicitAndPrecedesTarget(tc)
            identifiers={'robotenergy:PowerLimit','robotenergy:LegReach','robotenergy:Empty'};
            reasons={'battery_power_limit','leg_reach_limit','battery_depleted'};
            b=tc.Bundle; state=initialize_mpc_replay_state(b.cfg,0);
            for k=1:numel(identifiers)
                tc.verifyEqual(robotenergy.failureReason(identifiers{k}),reasons{k});
                out=struct('completed_horizon',false,'terminal_reason',reasons{k});
                [reason,done]=resolve_phase3_terminal(state,out,1,b.cfg);
                tc.verifyTrue(done); tc.verifyEqual(reason,reasons{k});
            end
            tc.verifyEmpty(robotenergy.failureReason('robotenergy:BadConfiguration'));
        end
        function legacyConsumerPreservesFailedAttemptAndRollsBack(tc)
            b=tc.Bundle; c=b.cfg; c.ENERGY.auxW=1e8;
            c.RUN.sim_snapshot_file=fullfile(fileparts(tc.Folder),'legacy.mat');
            reset_mpc_case_state();
            result=run_MPC_simulation(repmat(b.initial_R,4,1),0,.5,.5,c);
            tc.verifyFalse(result.feasible);
            tc.verifyEqual(result.fail_reason,'battery_power_limit');
            tc.verifyEqual(result.qp_failed_count,0);
            tc.verifyEqual(result.charge_total,0); tc.verifyEqual(result.current_duration,0);
            saved=load(c.RUN.sim_snapshot_file,'Sim');
            tc.verifyEqual(saved.Sim.t,0);
            tc.verifyEqual(saved.Sim.energy.elapsed,0);
            tc.verifyEmpty(saved.Sim.current_time);
            tc.verifyEqual(saved.Sim.energy_failure.attempted_time_s,.01);
            tc.verifyEqual(saved.Sim.energy_failure.identifier,'robotenergy:PowerLimit');
        end
        function legacyUnexpectedEnergyErrorRethrowsAfterSavingEvidence(tc)
            b=tc.Bundle; c=b.cfg; c.ENERGY=rmfield(c.ENERGY,'auxW');
            c.RUN.sim_snapshot_file=fullfile(fileparts(tc.Folder),'legacy.mat');
            reset_mpc_case_state();
            tc.verifyError(@()run_MPC_simulation(repmat(b.initial_R,4,1),0,.5,.5,c), ...
                'MATLAB:nonExistentField');
            saved=load(c.RUN.sim_snapshot_file,'Sim');
            tc.verifyEqual(saved.Sim.t,0); tc.verifyEqual(saved.Sim.energy.elapsed,0);
            tc.verifyEqual(saved.Sim.energy_failure.attempted_time_s,.01);
        end
        function invalidReachIsAnExplicitModelDomainError(tc)
            b=tc.Bundle; s=initialize_mpc_replay_state(b.cfg,0); p=get_params(0);
            x=s.Xt; x(19:21)=[5;5;5];
            f=struct('FSM',ones(4,1));
            tc.verifyError(@()robotenergy.loadStep(x,x,zeros(12,1),p,.01,0,f, ...
                s.knee_template,s.knee_parameters.Tmc,b.cfg.ENERGY),'robotenergy:LegReach');
        end
        function laterFailureKeepsPreviouslyAcceptedCharge(tc)
            b=tc.Bundle; c=b.cfg; state=initialize_mpc_replay_state(c,0);
            control=resolve_replay_action(localAction(b),state,c,b.lower_abs,b.upper_abs);
            control.reference_mode='position_continuous_v1';
            options=struct('duration_s',.02,'update_battery',true,'update_proxy',false);
            [before,normal]=simulate_mpc_horizon(state,control,c,options);
            tc.assertTrue(normal.completed_horizon);
            tc.assertGreaterThan(before.energy.chargeAs,0);
            % The configuration change is a fault injection at a saved prefix,
            % not a physical comparison or a qualified continuation.
            c.ENERGY.auxW=1e8;
            [after,failed]=simulate_mpc_horizon(before,control,c,options);
            tc.verifyEqual(failed.terminal_reason,"battery_power_limit");
            tc.verifyEqual(after.t,before.t);
            tc.verifyEqual(after.Xt,before.Xt); tc.verifyEqual(after.Ut,before.Ut);
            tc.verifyEqual(after.energy,before.energy);
            tc.verifyEqual(after.current_time,before.current_time);
            tc.verifyEqual(after.current_total,before.current_total);
            tc.verifyEqual(failed.charge_As,0); tc.verifyEqual(failed.energy_Wh,0);
            tc.verifyEqual(failed.failure_solver.energy_exception.attempted_state.t,.03,'AbsTol',1e-12);
        end
    end
end

function action=localAction(b)
action=[zeros(3,1);b.cfg.GAMMA_V_MISSION;b.cfg.GAMMA_A_MIN];
end

function row=localFirstRow(folder)
index=readtable(fullfile(folder,'mpc_steps','index.csv'),'TextType','string');
rows=read_exact_state_rows(fullfile(folder,index.file(1))); row=rows{1};
end

function localVerifyUncommitted(tc,row,identifier)
tc.verifyTrue(row.solver.success);
tc.verifyFalse(row.integrated); tc.verifyFalse(row.current_sample_committed);
tc.verifyEqual(row.time_after,row.time_before);
tc.verifyEqual(row.state_after,row.state_before);
tc.verifyEqual(row.solver.energy_exception.identifier,identifier);
tc.verifyEqual(row.solver.energy_exception.attempted_state.t,row.time_before+.01,'AbsTol',1e-12);
tc.verifyNotEmpty(row.qp_reference.file);
bad=row; bad.current_sample_committed=true;
tc.verifyError(@()validate_exact_state_row(bad,[]),'validate_exact_state_row:EnergyTransaction');
end
