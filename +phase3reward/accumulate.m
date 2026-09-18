function a = accumulate(a,row,dt)
%accumulate Reduce already-captured rows without retaining another state history.
    validateattributes(dt,{'double'},{'scalar','real','finite','positive'});
    validateattributes(row.row_id,{'double'},{'scalar','real','finite','positive','integer'});
    validateattributes(row.time_before,{'double'},{'scalar','real','finite','nonnegative'});
    validateattributes(row.time_after,{'double'},{'scalar','real','finite','nonnegative'});
    flags = [row.integrated,row.solver.success,row.current_sample_committed];
    assert(isreal(flags) && numel(flags)==3 && all(flags==0 | flags==1) && ...
        ~(row.integrated && ~row.solver.success) && ...
        ~(row.current_sample_committed && ~row.integrated), ...
        'phase3reward:Flags','Inconsistent capture flags.');
    if isempty(a)
        a = struct('dt',dt,'first_row',row.row_id,'last_row',row.row_id-1, ...
            'last_time',row.time_before,'previous_integrated',true,'attempts',0, ...
            'integrated_steps',0,'known',zeros(1,3),'finite_steps',zeros(1,3), ...
            'sample_count',0,'last_sample_time',NaN,'last_sample_A',NaN,'last_sample_row',NaN, ...
            'sampled_charge',0,'adjacent_charge',0,'adjacent_support',0, ...
            'gap_count',0,'gap_duration',0,'gap_charge',0);
    end
    tol = max(1e-10,64*eps(max(1,abs(row.time_after))));
    assert(a.dt==dt && row.row_id==a.last_row+1 && a.previous_integrated && ...
        all(isfinite([row.time_before,row.time_after])) && row.time_before>=0 && ...
        abs(row.time_before-a.last_time)<=tol && ...
        abs(row.time_after-row.time_before-dt*row.integrated)<=tol, ...
        'phase3reward:Continuity','Reward rows must be contiguous captured MPC attempts.');
    before = localHealth(row.health_before);
    after = localHealth(row.health_after);
    available = logical(row.integrated) & isfinite(before) & isfinite(after);
    a.known(available) = a.known(available)+dt*(0.5*before(available).^2+0.5*after(available).^2);
    a.finite_steps = a.finite_steps+double(available);
    a.integrated_steps = a.integrated_steps+double(row.integrated);
    if row.current_sample_committed
        t = row.current_sample_time;
        current = row.current_sample_A;
        assert(isscalar(t) && isscalar(current) && isreal([t,current]) && ...
            all(isfinite([t,current])) && abs(t-row.time_before)<=tol, ...
            'phase3reward:Current','Committed sample must be finite at the captured input time.');
        if a.sample_count>0
            interval = t-a.last_sample_time;
            assert(interval>0,'phase3reward:Current','Current sample time must increase.');
            charge = interval*(abs(a.last_sample_A)/2+abs(current)/2);
            a.sampled_charge = a.sampled_charge+charge;
            if row.row_id==a.last_sample_row+1
                a.adjacent_charge = a.adjacent_charge+charge;
                a.adjacent_support = a.adjacent_support+interval;
            else
                a.gap_count = a.gap_count+1;
                a.gap_duration = a.gap_duration+interval;
                a.gap_charge = a.gap_charge+charge;
            end
        end
        a.sample_count = a.sample_count+1;
        a.last_sample_time = t;
        a.last_sample_A = current;
        a.last_sample_row = row.row_id;
    end
    assert(all(isfinite([a.known,a.sampled_charge,a.adjacent_charge,a.gap_charge])), ...
        'phase3reward:Overflow','Observed objective contributions overflowed.');
    a.last_row = row.row_id;
    a.last_time = row.time_after;
    a.previous_integrated = logical(row.integrated);
    a.attempts = a.attempts+1;
end

function values = localHealth(health)
    values = nan(1,3);
    assert(isscalar(health.valid) && isreal(health.valid) && ...
        any(health.valid==[0,1]),'phase3reward:Health','Require an explicit health-valid flag.');
    if health.valid
        values = [health.components.orientation_error_rad,health.components.angular_velocity_norm,health.linear_velocity_error];
    end
    assert(numel(values)==3 && isreal(values) && ~any(values<0), ...
        'phase3reward:Health','Require nonnegative physical magnitudes.');
end
