package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class LazyVictorSPX extends VictorSPX implements IGreenMotor {

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double getLastSet() {
        System.out.println("getLastSet not implemented in LazyVictorSPX - returning 0");
        return 0;
    }
}
