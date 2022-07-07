package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;

public interface IGreenMotor extends IMotorControllerEnhanced {
    double getLastSet();
    String getName();
}
