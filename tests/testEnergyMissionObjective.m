classdef testEnergyMissionObjective < matlab.unittest.TestCase
    methods(Test)
        function physicalCurrentUnitAndReserve(tc)
            c = fixtureConfig(); s = energy_mission_scales(c);
            tc.verifyEqual(s.one_C_pack_current_A,2);
            tc.verifyEqual(s.pack_capacity_As,7200);
            tc.verifyEqual(s.available_charge_budget_As,5400,'AbsTol',1e-12);
        end
        function consumptionAndTimeHaveCorrectMonotonePreferences(tc)
            c = fixtureConfig(); w = fixtureWindow(0,100,0,40,0,10,'');
            r = compute_energy_mission_reward(w,c);
            moreCharge = w; moreCharge.charge_As=20; moreCharge.cumulative_charge_As=20;
            moreTime = w; moreTime.duration_s=200; moreTime.elapsed_time_s=200;
            tc.verifyLessThan(compute_energy_mission_reward(moreCharge,c),r);
            tc.verifyLessThan(compute_energy_mission_reward(moreTime,c),r);
        end
        function estimatedSocCannotCreateConsumptionReward(tc)
            c = fixtureConfig(); w = fixtureWindow(0,100,0,40,0,10,'');
            w.soc_start_pct=95; w.soc_end_pct=94;
            [r,a] = compute_energy_mission_reward(w,c);
            w.soc_end_pct=100;
            tc.verifyEqual(compute_energy_mission_reward(w,c),r);
            tc.verifyEqual(a.physical_soc_loss_fraction,10/7200);
        end
        function splittingWindowPreservesReturn(tc)
            c = fixtureConfig(); full = fixtureWindow(0,200,0,120,0,60,'');
            first = fixtureWindow(0,80,0,50,0,25,'');
            second = fixtureWindow(80,200,50,120,25,60,'');
            tc.verifyEqual(compute_energy_mission_reward(full,c), ...
                compute_energy_mission_reward(first,c)+compute_energy_mission_reward(second,c), ...
                'AbsTol',1e-14);
        end
        function reversingProgressCannotEarnRepeatedCredit(tc)
            c = fixtureConfig(); c.ENERGY_RL.time_weight=0; c.ENERGY_RL.charge_weight=1;
            forward = fixtureWindow(0,10,0,40,0,0,'');
            reverse = fixtureWindow(10,20,40,0,0,0,'');
            tc.verifyEqual(compute_energy_mission_reward(forward,c)+ ...
                compute_energy_mission_reward(reverse,c),0,'AbsTol',1e-14);
            overshoot = fixtureWindow(0,10,300,330,0,0,'');
            tc.verifyEqual(compute_energy_mission_reward(overshoot,c),20/320,'AbsTol',1e-14);
        end
        function completionDominatesFailedEpisodeAcrossPhysicalBudget(tc)
            c = fixtureConfig(); s=energy_mission_scales(c);
            worstSuccess = fixtureWindow(0,600,0,320,0,s.available_charge_budget_As,'mission_complete');
            lateFailure = fixtureWindow(0,0.01,0,319.999,0,0,'time_limit');
            tc.verifyEqual(compute_energy_mission_reward(worstSuccess,c),3,'AbsTol',1e-12);
            tc.verifyLessThan(compute_energy_mission_reward(lateFailure,c),0);
        end
        function zeroDurationFailureChargesOnlyTerminal(tc)
            c=fixtureConfig(); w=fixtureWindow(100,100,20,20,5,5,'battery_power_limit');
            tc.verifyEqual(compute_energy_mission_reward(w,c),-1);
            w.distance_end_m=21;
            tc.verifyError(@()compute_energy_mission_reward(w,c),'energyMission:ZeroDuration');
        end
        function declaredBudgetsAndDiscountAreEnforced(tc)
            c=fixtureConfig(); w=fixtureWindow(0,601,0,320,0,1,'mission_complete');
            tc.verifyError(@()compute_energy_mission_reward(w,c),'energyMission:Budget');
            w=fixtureWindow(0,600,0,320,0,5401,'mission_complete');
            tc.verifyError(@()compute_energy_mission_reward(w,c),'energyMission:Budget');
            c.TRAIN.discount_factor=.99;
            tc.verifyError(@()energy_mission_scales(c),'energyMission:Discount');
        end
        function unknownTerminalAndFalseCompletionAreRejected(tc)
            c=fixtureConfig(); w=fixtureWindow(0,10,0,1,0,1,'mission_complete');
            tc.verifyError(@()compute_energy_mission_reward(w,c),'energyMission:Completion');
            w.terminal_reason='made_up';
            tc.verifyError(@()compute_energy_mission_reward(w,c),'energyMission:Terminal');
        end
        function currentTailsRemainVisibleAndTrueSocIsNotExposed(tc)
            [c,state,row,legacy] = observationFixture();
            w=struct('Ieq_window',4);
            [o,a]=build_energy_mission_observation(state,legacy,row,c,w);
            idx=a.schema.names=="previous_pack_current_1C";
            oneC=c.BATTERY.n_parallel*c.BATTERY.C_nom_Ah;
            tc.verifyEqual(o(idx),asinh(4/oneC),'AbsTol',1e-14);
            tc.verifyFalse(a.true_soc_exposed);
            changed=state; changed.energy.soc=.21; changed.energy.rc=ones(3,1)*99;
            row.state_after=changed;
            tc.verifyEqual(build_energy_mission_observation(changed,legacy,row,c,w),o);
            tc.verifyFalse(any(a.clipped));
        end
        function forecastUsesOnlyObservedChargeAndDistance(tc)
            [c,state,row,legacy] = observationFixture();
            [~,a]=build_energy_mission_observation(state,legacy,row,c,struct('Ieq_window',.5));
            tc.verifyTrue(a.forecast_available);
            tc.verifyEqual(a.projected_remaining_charge_As,10/40*280);
            capacity=3600*c.BATTERY.n_parallel*c.BATTERY.C_nom_Ah;
            tc.verifyEqual(a.projected_reserve_surplus_As,capacity*(.94-.2)-70,'AbsTol',1e-10);
        end
        function resetForecastAndElectricalMeasurementAvailabilityAreExplicit(tc)
            b=setup_RL_MPC(); c=b.cfg; f=fixtureConfig(); c.ENERGY_RL=f.ENERGY_RL;
            c.TRAIN.discount_factor=1;
            state=initialize_mpc_replay_state(c,0);
            state.decision_bookkeeping=struct('initial_position_x',state.Xt(1));
            [o,a]=build_energy_mission_observation(state,zeros(19,1),struct(),c);
            tc.verifyTrue(all(isfinite(o)));
            tc.verifySize(o,[87,1]);
            tc.verifyFalse(a.forecast_available);
            tc.verifyTrue(isnan(a.projected_remaining_charge_As));
            tc.verifyEqual(o(a.schema.names=="has_charge_forecast"),0);
            tc.verifyEqual(o(a.schema.names=="has_voltage_measurement"),0);
            tc.verifyEqual(o(a.schema.names=="has_drive_headroom"),0);
        end
        function nonzeroTimeRequiresActualDecisionCurrent(tc)
            [c,state,row,legacy]=observationFixture();
            tc.verifyError(@()build_energy_mission_observation(state,legacy,row,c), ...
                'energyMission:CurrentHistory');
        end
    end
end

function c=fixtureConfig()
c=struct('MISSION_DURATION',600,'MISSION',struct('D_TARGET_M',320), ...
    'BATTERY',struct('use_pack_sizing',false,'n_series',20,'n_parallel',1, ...
    'C_nom_Ah',2,'SOC_init',.95,'terminal_margin',.2), ...
    'TRAIN',struct('discount_factor',1), ...
    'ENERGY_RL',struct('reference_charge_As',165,'time_weight',.5,'charge_weight',.5));
end

function w=fixtureWindow(t0,t1,d0,d1,q0,q1,terminal)
w=struct('duration_s',t1-t0,'charge_As',q1-q0, ...
    'elapsed_time_s',t1,'cumulative_charge_As',q1, ...
    'distance_start_m',d0,'distance_end_m',d1,'terminal_reason',terminal);
end

function [c,state,row,legacy]=observationFixture()
b=setup_RL_MPC(); c=b.cfg; f=fixtureConfig(); c.ENERGY_RL=f.ENERGY_RL;
c.TRAIN.discount_factor=1;
state=initialize_mpc_replay_state(c,0); state.t=100;
state.Xt(1)=40; state.energy.estimate(1)=.94; state.energy.chargeAs=10;
state.decision_bookkeeping=struct('initial_position_x',0);
state.fsm_internal_state=struct('FSM',[1;2;2;1], ...
    'Ta',99.8*ones(4,1),'Tb',100.1*ones(4,1));
state.battery.pack_voltage_V=80; state.battery.drive_voltage_headroom_V=10;
row=struct('Xd',state.Xt,'time_after',state.t,'state_after',state, ...
    'action_execution',struct('candidate_action',[0;0;0;.45;.2], ...
    'applied_action',[0;0;0;.32;.2]), ...
    'solver',struct('iterations',14,'wall_time_s',.015, ...
    'classification','solver_success','diagnostics',struct('inequality_margin_min',0)));
legacy=zeros(19,1);
end
