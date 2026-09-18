function [reward,info] = compute_phase3_reward_candidate(window,execution,exposure,before,after,cfg,policy)
%compute_phase3_reward_candidate Captured, opt-in finite reward with full audit.
    [scales,scaleAudit] = phase3reward.scales(cfg);
    n = numel(before.current_time);
    times = after.current_time(:);
    currents = after.current_total(:);
    assert(numel(times)>=n && numel(currents)==numel(times) && ...
        numel(before.current_total)==n && ...
        isequal(times(1:n),before.current_time(:)) && ...
        isequal(currents(1:n),before.current_total(:)), ...
        'phase3reward:History','Current history must remain append-only.');
    previous = [];
    if n>0
        previous = [before.current_time(end),before.current_total(end)];
    end
    p = get_params(before.gait);
    charge = phase3reward.bookedCharge(previous,times(n+1:end),currents(n+1:end), ...
        before.t,after.t,p.simTimeStep);
    assert(abs(charge.booked_charge_As-window.charge_As)<1e-8 && ...
        abs(after.t-before.t-window.duration_s)<1e-8, ...
        'phase3reward:Accounting','Saved decision accounting differs from committed current history.');
    health = struct('integrated_duration_s',exposure.integrated_duration_s);
    names = {'orientation','omega','velocity_error'};
    for k = 1:3
        health.([names{k} '_squared_integral_observed']) = exposure.full_squared_integrals(k);
        health.([names{k} '_finite_coverage_s']) = exposure.finite_coverage_s(k);
    end
    components = phase3reward.components(window,execution,health,scales);
    [reward,info] = phase3reward.scoreCoverage(components,exposure,policy);
    info.components = components;
    info.scale_audit = scaleAudit;
    info.charge_audit = charge;
    [info.comparison_bridge_reward,info.comparison_bridge_info] = compute_phase3_reward_bridge(window,cfg);
end
