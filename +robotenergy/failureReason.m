function reason = failureReason(identifier)
%failureReason Classify expected model-domain failures without hiding bugs.
switch char(identifier)
    case 'robotenergy:PowerLimit'
        reason = 'battery_power_limit';
    case 'robotenergy:LegReach'
        reason = 'leg_reach_limit';
    case 'robotenergy:Empty'
        reason = 'battery_depleted';
    otherwise
        reason = '';
end
end
