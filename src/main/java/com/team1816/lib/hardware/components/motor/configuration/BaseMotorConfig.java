package com.team1816.lib.hardware.components.motor.configuration;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;

public class BaseMotorConfig extends BaseTalonConfiguration {

    public BaseMotorConfig(FeedbackDevice defaultFeedbackDevice) {
        super(defaultFeedbackDevice);
    }
}
