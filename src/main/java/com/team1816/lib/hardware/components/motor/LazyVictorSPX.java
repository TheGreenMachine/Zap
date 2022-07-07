package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyVictorSPX extends VictorSPX implements IGreenMotor {

    protected String name = "";

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public LazyVictorSPX(int deviceNumber, String motorName) {
        super(deviceNumber);
        name = motorName;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configSupplyCurrentLimit not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: setStatusFramePeriod not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public double getOutputCurrent() {
        DriverStation.reportWarning(
            "method: getOutputCurrent not implemented for LazyVictorSPX",
            false
        );
        return 0;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configReverseLimitSwitchSource not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public double getLastSet() {
        DriverStation.reportWarning(
            "getLastSet not implemented in LazyVictorSPX - returning 0",
            false
        );
        return 0;
    }

    @Override
    public String getName() {
        return name;
    }
}
