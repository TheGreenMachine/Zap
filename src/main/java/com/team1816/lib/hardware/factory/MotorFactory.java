package com.team1816.lib.hardware.factory;

import static com.team1816.lib.subsystems.Subsystem.factory;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.motor.*;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.*;

/**
 * A class to create Falcon (TalonFX), TalonSRX, VictorSPX, SparkMax and GhostMotor objects.
 * Based on FRC Team 254 The Cheesy Poof's 2018 TalonSRXFactory
 */
public class MotorFactory {

    private static final int kTimeoutMs = RobotBase.isSimulation() ? 0 : 100;
    private static final int kTimeoutMsLONG = RobotBase.isSimulation() ? 0 : 200;


    // Factory default motor configs.

    public static NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    public static double NEUTRAL_DEADBAND = 0.04;
    public static boolean ENABLE_CURRENT_LIMIT = false;
    public static boolean ENABLE_SOFT_LIMIT = false;
    public static boolean ENABLE_LIMIT_SWITCH = false;
    public static int FORWARD_SOFT_LIMIT = 0;
    public static int REVERSE_SOFT_LIMIT = 0;
    public static int CONTROL_FRAME_PERIOD_MS = 5;
    public static int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 1;
    public static double OPEN_LOOP_RAMP_RATE = 0.0;
    public static double CLOSED_LOOP_RAMP_RATE = 0.0;


    // Create a CANTalon with the default (out of the box) configuration.
    public static IGreenMotor createDefaultTalon(
        int id,
        String name,
        boolean isFalcon,
        SubsystemConfig subsystems,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId,
        String canBus
    ) {
        return createTalon(
            id,
            name,
            isFalcon,
            subsystems,
            pidConfigList,
            remoteSensorId,
            canBus
        );
    }

    public static IGreenMotor createFollowerTalon(
        int id,
        String name,
        boolean isFalcon,
        IGreenMotor main,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        String canBus
    ) {
        final IGreenMotor talon = createTalon(
            id,
            name,
            isFalcon,
            subsystem,
            pidConfigList,
            -1, // never can have a remote sensor on Follower,
            canBus
        );
        System.out.println(
            "Slaving talon on " + id + " to talon on " + main.getDeviceID()
        );
        talon.follow(main);
        return talon;
    }

    private static IGreenMotor createTalon(
        int id,
        String name,
        boolean isFalcon,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId,
        String canBus
    ) {
        IGreenMotor talon = isFalcon
            ? new LazyTalonFX(id, name, canBus)
            : new LazyTalonSRX(id, name);
        configMotor(
            talon,
            name,
            isFalcon,
            subsystem,
            pidConfigList,
            remoteSensorId
        );

        return talon;
    }

    public static IGreenMotor createGhostMotor(
        int maxVelTicks100ms,
        int absInitOffset,
        String name,
        SubsystemConfig subsystem
        ) {
        IGreenMotor motor = new GhostMotor(maxVelTicks100ms, absInitOffset, name);
        configMotor(motor, name, subsystem, null, -1);
        return motor;
    }

    public static IGreenMotor createDefaultVictor(int id, String name) {
        return createVictor(id, name);
    }

    public static IGreenMotor createFollowerVictor(
        int id,
        String name,
        IGreenMotor main
    ) {
        final IGreenMotor victor = createVictor(id, name);
        System.out.println(
            "Slaving victor on " + id + " to talon on " + main.getDeviceID()
        );
        victor.follow(main);
        return victor;
    }

    // This is currently treating a VictorSPX, which implements IMotorController as an IGreenMotor, which implements IMotorControllerEnhanced
    public static IGreenMotor createVictor(int id, String name) {
        IGreenMotor victor = new LazyVictorSPX(id, name);

        victor.configReverseLimitSwitchSource(
            LimitSwitchSource.Deactivated,
            LimitSwitchNormal.NormallyOpen,
            kTimeoutMs
        );
        return victor;
    }
    public static IGreenMotor createSpark(
        int id,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        return new LazySparkMax(id, name);
    }

    public static IGreenMotor createSpark(
        int id,
        String name,
        SubsystemConfig subsystem
    ) {
        return new LazySparkMax(id, name);
    }

    public static CANCoder createCanCoder(int canCoderID, boolean invertCanCoder) {
        CANCoder canCoder = new CANCoder(canCoderID);
        if (factory.getConstant("resetFactoryDefaults", 0) > 0) {
            canCoder.configFactoryDefault(kTimeoutMs);
        }
        canCoder.configAllSettings(configureCanCoder(invertCanCoder), kTimeoutMsLONG);
        return canCoder;
    }

    private static void configMotor(
        IGreenMotor motor,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        BaseTalonConfiguration talonConfiguration;

        // type of configuration (based on motor type)
        if (motor instanceof TalonFX) {
            talonConfiguration = new TalonFXConfiguration();
        } else if (motor instanceof TalonSRX) {
            talonConfiguration = new TalonSRXConfiguration();
        } else {
            return;
        }

        // setting pid
        if (pidConfigList != null) {
            pidConfigList.forEach(
                (slot, slotConfig) -> {
                    switch (slot.toLowerCase()) {
                        case "slot0":
                            talonConfiguration.slot0 = toSlotConfiguration(slotConfig);
                            break;
                        case "slot1":
                            talonConfiguration.slot1 = toSlotConfiguration(slotConfig);
                            break;
                        case "slot2":
                            talonConfiguration.slot2 = toSlotConfiguration(slotConfig);
                            break;
                        case "slot3":
                            talonConfiguration.slot3 = toSlotConfiguration(slotConfig);
                            break;
                    }
                }
            );
        }
        motor.selectProfileSlot(0, 0);

        // binding remote sensors to respective motors
        if (remoteSensorId >= 0) {
            talonConfiguration.primaryPID.selectedFeedbackSensor =
                FeedbackDevice.RemoteSensor0;
            talonConfiguration.remoteFilter0.remoteSensorDeviceID = remoteSensorId;
            talonConfiguration.remoteFilter0.remoteSensorSource =
                RemoteSensorSource.CANCoder;
        } else {
            talonConfiguration.primaryPID.selectedFeedbackSensor =
                (motor instanceof TalonFX)
                    ? FeedbackDevice.IntegratedSensor
                    : FeedbackDevice.CTRE_MagEncoder_Relative;
        }

        // for newly attached motors only
        if (factory.getConstant("resetFactoryDefaults", 0) > 0) {
            System.out.println("Resetting motor factory defaults");
            motor.configFactoryDefault(kTimeoutMs);
            talonConfiguration.forwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
            talonConfiguration.forwardSoftLimitEnable = ENABLE_SOFT_LIMIT;

            talonConfiguration.reverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;
            talonConfiguration.reverseSoftLimitEnable = ENABLE_SOFT_LIMIT;
            talonConfiguration.nominalOutputForward = 0;
            talonConfiguration.nominalOutputReverse = 0;
            talonConfiguration.neutralDeadband = NEUTRAL_DEADBAND;

            talonConfiguration.peakOutputForward = 1.0;
            talonConfiguration.peakOutputReverse = -1.0;

            talonConfiguration.velocityMeasurementWindow =
                VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

            talonConfiguration.openloopRamp = OPEN_LOOP_RAMP_RATE;
            talonConfiguration.closedloopRamp = CLOSED_LOOP_RAMP_RATE;
            if (talonConfiguration instanceof TalonFXConfiguration) {
                ((TalonFXConfiguration) talonConfiguration).supplyCurrLimit =
                    new SupplyCurrentLimitConfiguration(
                        ENABLE_CURRENT_LIMIT,
                        40,
                        80,
                        1
                    );
            } else {
                ((TalonSRXConfiguration) talonConfiguration).peakCurrentLimit = 80;
                ((TalonSRXConfiguration) talonConfiguration).peakCurrentDuration = 1;
                ((TalonSRXConfiguration) talonConfiguration).continuousCurrentLimit = 40;
            }

            talonConfiguration.clearPositionOnLimitF = false;
            talonConfiguration.clearPositionOnLimitR = false;

            talonConfiguration.enableOptimizations = true;

            motor.overrideLimitSwitchesEnable(ENABLE_LIMIT_SWITCH);

            motor.setNeutralMode(NEUTRAL_MODE);

            motor.setControlFramePeriod(
                ControlFrame.Control_3_General,
                CONTROL_FRAME_PERIOD_MS
            );
        }

        // applying configs to motor
        motor.configAllSettings(talonConfiguration, kTimeoutMs);

        // inverting
        motor.setInverted(subsystem.invertMotor.contains(name));
        motor.setSensorPhase(subsystem.invertSensorPhase.contains(name));
    }

    private static CANCoderConfiguration configureCanCoder(boolean invertCanCoder) {
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.sensorDirection = invertCanCoder;
        canCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        return canCoderConfig;
    }

    private static SlotConfiguration toSlotConfiguration(
        PIDSlotConfiguration pidConfiguration
    ) {
        SlotConfiguration slotConfig = new SlotConfiguration();
        if (pidConfiguration != null) {
            if (pidConfiguration.kP != null) slotConfig.kP = pidConfiguration.kP;
            if (pidConfiguration.kI != null) slotConfig.kI = pidConfiguration.kI;
            if (pidConfiguration.kD != null) slotConfig.kD = pidConfiguration.kD;
            if (pidConfiguration.kP != null) slotConfig.kF = pidConfiguration.kF;
            if (pidConfiguration.iZone != null) slotConfig.integralZone =
                pidConfiguration.iZone;
            if (
                pidConfiguration.allowableError != null
            ) slotConfig.allowableClosedloopError = pidConfiguration.allowableError;
        }
        return slotConfig;
    }

    public enum MotorType {
        TalonSRX,
        Falcon,
        SparkMax,
        Ghost
    }
}
