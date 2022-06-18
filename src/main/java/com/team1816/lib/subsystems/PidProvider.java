package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.PIDSlotConfiguration;

public interface PidProvider {
    PIDSlotConfiguration getPIDConfig();

    default String pidToString() {
        return String.format(
            "kP = %f, kI = %f, kD = %f, kF = %f",
            getPIDConfig().kP,
            getPIDConfig().kI,
            getPIDConfig().kD,
            getPIDConfig().kF
        );
    }
}
