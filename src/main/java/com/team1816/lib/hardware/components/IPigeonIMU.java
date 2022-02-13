package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;

public interface IPigeonIMU {
    double getFusedHeading();

    ErrorCode setYaw(double angleDeg);

    ErrorCode setFusedHeading(double angleDeg);

    ErrorCode setAccumZAngle(double angleDeg);

    boolean hasResetOccurred();

    ErrorCode configFactoryDefault();
}
