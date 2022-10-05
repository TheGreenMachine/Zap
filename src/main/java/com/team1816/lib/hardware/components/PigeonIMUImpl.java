package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonIMUImpl extends PigeonIMU implements IPigeonIMU {

    private long m_handle = 0l;

    public PigeonIMUImpl(int id) {
        super(id);
        m_handle = super.getHandle();
        System.out.println("PIGEON HANDLE: " + m_handle);
    }

    @Override
    public double getYaw() {
        return super.getYaw();
    }

    @Override
    public double[] getAcceleration() {
        short[] accel = new short[3];
        getBiasedAccelerometer(accel);
        return new double[] { accel[0], accel[1], accel[2] };
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
