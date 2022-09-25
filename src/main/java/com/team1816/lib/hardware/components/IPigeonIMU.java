package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public interface IPigeonIMU {
    double getYaw();

    ErrorCode setYaw(double angleDeg);

    ErrorCode setFusedHeading(double angleDeg);

    ErrorCode setAccumZAngle(double angleDeg);

    boolean hasResetOccurred();

    ErrorCode configFactoryDefault();

    ErrorCode setStatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs);
}
