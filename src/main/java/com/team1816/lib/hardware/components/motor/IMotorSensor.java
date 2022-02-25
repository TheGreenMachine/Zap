package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;

public interface IMotorSensor {
    int getQuadraturePosition();
    int getPulseWidthPosition();
    ErrorCode setQuadraturePosition(int newPosition);
}
