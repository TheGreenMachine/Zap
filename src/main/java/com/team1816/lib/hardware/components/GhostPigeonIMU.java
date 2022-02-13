package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

public class GhostPigeonIMU implements IPigeonIMU {

    public GhostPigeonIMU(int id) {}

    public GhostPigeonIMU(IMotorControllerEnhanced motor) {}

    @Override
    public double getFusedHeading() {
        return 0;
    }

    @Override
    public ErrorCode setYaw(double angleDeg) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setFusedHeading(double angleDeg) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setAccumZAngle(double angleDeg) {
        return ErrorCode.OK;
    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return ErrorCode.OK;
    }
}
