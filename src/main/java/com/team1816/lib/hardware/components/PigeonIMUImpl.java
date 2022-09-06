package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonImuJNI;

public class PigeonIMUImpl extends PigeonIMU implements IPigeonIMU {

    public PigeonIMUImpl(int id) {
        super(id);
    }

    @Override
    public double getYaw() {
        return super.getYaw();
    }

    public double[] getAcceleration() {
        short[] accel = new short[3];
        long handle = 0l;
        PigeonImuJNI.JNI_GetBiasedAccelerometer(handle, accel);
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
