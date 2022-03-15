package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.*;
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
        return createSpark(id);
    }

    public static IMotorControllerEnhanced createSpark(
        int id
    ) {
        return new LazySparkMax(id);
    }
}
