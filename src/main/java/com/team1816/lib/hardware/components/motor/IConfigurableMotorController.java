package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;

public interface IConfigurableMotorController extends IGreenMotor {
    ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs);

    ErrorCode configFactoryDefault(int timeoutMs);
}
