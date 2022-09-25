package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team1816.lib.hardware.components.motor.IGreenMotor;

public class GhostPigeonIMU implements IPigeonIMU {

    double dummyYaw;

    public GhostPigeonIMU(int id) {
        dummyYaw = 0;
    }

    public GhostPigeonIMU(IGreenMotor motor) {}

    @Override
    public double getYaw() {
        return dummyYaw;
    }

    @Override
    public ErrorCode setYaw(double angleDeg) {
        dummyYaw = angleDeg;
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

    @Override
    public ErrorCode setStatusFramePeriod(
        PigeonIMU_StatusFrame statusFrame,
        int periodMs
    ) {
        return ErrorCode.OK;
    }
}
