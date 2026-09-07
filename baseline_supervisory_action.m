function action = baseline_supervisory_action(policy, time, cfg)
%baseline_supervisory_action Reproducible candidates; normal governor still applies.
    switch string(policy)
        case {"mission_average","fixed_R"}
            velocity = cfg.MISSION.D_TARGET_M/cfg.MISSION_DURATION;
            gamma = (velocity-cfg.V_MIN)/(cfg.V_MAX-cfg.V_MIN);
        case "fast_start"
            gamma = cfg.GAMMA_V_MAX;
        case "slow_start"
            if time < 100
                gamma = 0.2;
            else
                gamma = 0.4;
            end
        case "hand_ramp"
            gamma = 0.2+0.25*min(time/300,1);
        case "conservative"
            gamma = 0.2;
        otherwise
            error('baseline_supervisory_action:UnknownPolicy','Unknown baseline %s.',policy);
    end
    action = [zeros(3,1);gamma;cfg.GAMMA_A_MIN];
end
