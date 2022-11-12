package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonIMUImpl extends PigeonIMU implements IPigeonIMU {

    public PigeonIMUImpl(int id) {
        super(id);
    }

    @Override
    public double getYaw() {
        return super.getYaw();
    }

    @Override
    public ErrorCode setYaw(double angleDeg) {
        return super.setYaw(angleDeg);
    }

    @Override
    public ErrorCode setFusedHeading(double angleDeg) {
        return super.setYaw(angleDeg);
    }

    @Override
    public ErrorCode setAccumZAngle(double angleDeg) {
        return super.setAccumZAngle(angleDeg);
    }

    @Override
    public boolean hasResetOccurred() {
        return super.hasResetOccurred();
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return super.configFactoryDefault();
    }
}
