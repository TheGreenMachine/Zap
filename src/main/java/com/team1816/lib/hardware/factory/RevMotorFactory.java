package com.team1816.lib.hardware.factory;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.SubsystemConfig;
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

    public static IMotorControllerEnhanced createSpark(
        int id,
        String name,
        SubsystemConfig subsystems,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        LazySparkMax sparkMax = new LazySparkMax(id);
        if (pidConfigList != null) {
            pidConfigList.forEach(
                (slot, slotConfig) -> {
                    switch (slot.toLowerCase()) {
                        case "slot0":
                            break;
                        case "slot1":
                            break;
                        case "slot2":
                            break;
                        case "slot3":
                            break;
                    }
                }
            );
        }
        // sparkMax remote sensor
        return sparkMax;
    }
}
