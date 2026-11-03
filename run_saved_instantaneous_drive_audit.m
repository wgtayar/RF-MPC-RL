function report = run_saved_instantaneous_drive_audit(inputRoot,outputRoot)
%RUN_SAVED_INSTANTANEOUS_DRIVE_AUDIT Reinterpret saved states, never rerun MPC.
% Evaluates both sides within each original held-reference interval. Reported
% maxima are sampled lower bounds on continuous-time peaks; no percentile is
% inferred from these unequally representative endpoint samples.
source=bootstrap_RF_MPC_RL();
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'driveAudit:Exists','Use a new output root.');
mkdir(outputRoot);
meta=struct('schema','saved_instantaneous_drive_audit_v1', ...
    'source_sha',gitValue(source,'rev-parse HEAD'), ...
    'source_branch',gitValue(source,'branch --show-current'), ...
    'source_dirty',~isempty(gitValue(source,'status --porcelain')), ...
    'input_root',inputRoot,'matlab',version,'training',false, ...
    'learning_rates_used',[],'exploration_noise_used',[], ...
    'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'interpretation','New analytic derivative applied to preserved trajectories; not an original recorded drive measurement.');
cases={'short_slow','mission_average','mission_fast'};
results=cell(size(cases));
for c=1:numel(cases)
    root=fullfile(inputRoot,cases{c});
    saved=load(fullfile(root,'manifest','configuration.mat'),'cfg'); cfg=saved.cfg;
    manifest=load(fullfile(root,'manifest','run.mat'),'manifest');
    index=readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
    values=zeros(24*sum(index.row_count),13); n=0;
    for f=1:height(index)
        rows=read_exact_state_rows(fullfile(root,index.file(f)));
        for k=1:numel(rows)
            row=rows{k}; if ~row.integrated, continue; end
            saved=load(fullfile(root,row.snapshot_reference.file),'snapshot');
            p=saved.snapshot.parameters; state=saved.snapshot.state;
            fsm=row.state_after.fsm_internal_state;
            for endpoint=0:1
                if endpoint==0
                    x=row.Xt_qp; time=row.time_before;
                else
                    x=row.state_after.Xt; time=row.time_after;
                end
                [~,p.p_ext]=fcn_get_disturbance(time,p);
                % External force changes only acceleration components, which
                % the IK derivative does not consume. This is not a new load case.
                dx=dynamics_SRB(time,x,row.state_after.Ut,row.Xd,zeros(3,1),p);
                m=robotenergy.instantaneousLoad(x,dx,row.state_after.Ut,p,time,fsm, ...
                    state.knee_template,state.knee_parameters.Tmc,cfg.ENERGY);
                for leg=1:4
                    for axis=1:3
                        n=n+1;
                        values(n,:)=[row.row_id,time,endpoint,leg,axis, ...
                            m.jointTorque(axis,leg),m.jointSpeed(axis,leg), ...
                            m.motorTorque(axis,leg),m.phaseCurrent(axis,leg), ...
                            m.motorSpeed(axis,leg),m.motorVoltage(axis,leg),m.watts, ...
                            m.jacobianReciprocalCondition(leg)];
                    end
                end
            end
        end
    end
    T=array2table(values(1:n,:),'VariableNames',{'row','time_s','endpoint','leg','axis', ...
        'joint_torque_Nm','joint_speed_rad_s','motor_torque_Nm','phase_current_A', ...
        'motor_speed_rad_s','motor_voltage_V','total_demand_W','jacobian_rcond'});
    file=[cases{c} '.csv']; writetable(T,fullfile(outputRoot,file));
    [maximum,peakRow]=max(abs(T.motor_voltage_V));
    results{c}=struct('case',cases{c},'original_source_sha',manifest.manifest.source_sha, ...
        'configuration_sha256',sha256_file(fullfile(root,'manifest','configuration.mat')), ...
        'seed',cfg.RNG_SEED,'reward_version',cfg.PHASE3.reward_version, ...
        'rows',height(T),'file',file,'sha256',sha256_file(fullfile(outputRoot,file)), ...
        'maximum_sampled_voltage_V',maximum,'peak_sample',table2struct(T(peakRow,:)), ...
        'maximum_sampled_motor_torque_Nm',max(abs(T.motor_torque_Nm)), ...
        'maximum_sampled_phase_current_A',max(abs(T.phase_current_A)), ...
        'maximum_sampled_total_demand_W',max(T.total_demand_W), ...
        'minimum_jacobian_rcond',min(T.jacobian_rcond));
end
report=struct('metadata',meta,'cases',{results}, ...
    'scope','Endpoint diagnostic only: does not establish full-mission time-percentiles or a continuous peak upper bound.');
f=fopen(fullfile(outputRoot,'audit.json'),'w'); assert(f>=0);
clean=onCleanup(@()fclose(f)); fprintf(f,'%s\n',jsonencode(report,'PrettyPrint',true));
end
function value=gitValue(source,args)
[status,value]=system(sprintf('git -C "%s" %s',source,args));
assert(status==0,'driveAudit:Git','Cannot record source provenance.'); value=strtrim(value);
end
