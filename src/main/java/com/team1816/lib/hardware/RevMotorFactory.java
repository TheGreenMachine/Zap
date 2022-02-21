package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.team1816.lib.hardware.components.motor.*;

import java.util.Map;

public class RevMotorFactory {

    public static IMotorControllerEnhanced createDefaultSpark(
           int id,
           String name,
           SubsystemConfig subsystems,
           Map<String, PIDSlotConfiguration> pidConfigList,
           int remoteSensorId
    ) {
        return createSpark(
            id,
            name,
            subsystems,
            pidConfigList
        );
    }

    private static IMotorControllerEnhanced createSpark(
         int id,
         String name,
         SubsystemConfig subsystem,
         Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        return new LazySparkMax(id);
    }

}
