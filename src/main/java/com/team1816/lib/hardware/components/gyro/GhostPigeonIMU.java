package com.team1816.lib.hardware.components.gyro;

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
    public double[] getAcceleration() {
        double[] accel = new double[] { 0d, 0d, 9.8d };
        return accel;
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
