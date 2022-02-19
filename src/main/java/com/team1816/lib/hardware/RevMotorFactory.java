package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.team1816.lib.hardware.components.motor.*;

import java.util.Map;

public class RevMotorFactory {

    public static class Configuration {

        public CANSparkMax.IdleMode IDLE_MODE = CANSparkMax.IdleMode.kCoast;
        // This is factory default.
        public double NEUTRAL_DEADBAND = 0.04;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 20;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 160;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 160;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 160;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
            VelocityMeasPeriod.Period_50Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 1;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final RevMotorFactory.Configuration kDefaultConfiguration = new RevMotorFactory.Configuration();

    public static IMotorControllerEnhanced createDefaultSpark(
           int id,
           String name,
           boolean isSparkmax,
           SubsystemConfig subsystems,
           Map<String, PIDSlotConfiguration> pidConfigList,
           int remoteSensorId
    ) {
        return createSpark(
            id,
            name,
            kDefaultConfiguration,
            isSparkmax,
            subsystems,
            pidConfigList,
            remoteSensorId
        );
    }

    private static IMotorControllerEnhanced createSpark(
         int id,
         String name,
         RevMotorFactory.Configuration config,
         boolean isSparkmax,
         SubsystemConfig subsystem,
         Map<String, PIDSlotConfiguration> pidConfigList,
         int remoteSensorId
    ) {
        if(!isSparkmax){
            System.out.println("spark motor isn't sparkMAX! NO working config - defaulting to sparkMAX!!!!!");
        }
        // for now assuming that it will always be a sparkMAX
        LazySparkMax spark = new LazySparkMax(id);
        configureMotorController(
            spark,
            name,
            config,
            isSparkmax,
            subsystem,
            pidConfigList,
            remoteSensorId
        );
        System.out.println("created collector");

        return spark;
    }

    public static IMotorControllerEnhanced createGhostRevController(int maxTicks){
        System.out.println("no ghost functionality for rev motors yet!");
        return new GhostRevMotorController();
    }

    private static void configureMotorController(
        LazySparkMax motor, // going to be an interface of some sort
        String name,
        RevMotorFactory.Configuration config,
        boolean isFalcon,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        // I'm assuming we actually need to make this properly configure with REV's equivalent of BaseTalonConfiguration

//        BaseTalonConfiguration talonConfiguration;
//
//        if (motor instanceof TalonFX) {
//            talonConfiguration = new TalonFXConfiguration();
//        } else if (motor instanceof TalonSRX) {
//            talonConfiguration = new TalonSRXConfiguration();
//        } else {
//            return;
//        }
//
//        talonConfiguration.forwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
//        talonConfiguration.forwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
//
//        talonConfiguration.reverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
//        talonConfiguration.reverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
//
//        if (pidConfigList != null) {
//            pidConfigList.forEach(
//                (slot, slotConfig) -> {
//                    switch (slot.toLowerCase()) {
//                        case "slot0":
//                            talonConfiguration.slot0 = toSlotConfiguration(slotConfig);
//                            break;
//                        case "slot1":
//                            talonConfiguration.slot1 = toSlotConfiguration(slotConfig);
//                            break;
//                        case "slot2":
//                            talonConfiguration.slot2 = toSlotConfiguration(slotConfig);
//                            break;
//                        case "slot3":
//                            talonConfiguration.slot3 = toSlotConfiguration(slotConfig);
//                            break;
//                    }
//                }
//            );
//        }
//        talonConfiguration.nominalOutputForward = 0;
//        talonConfiguration.nominalOutputReverse = 0;
//        talonConfiguration.neutralDeadband = config.NEUTRAL_DEADBAND;
//
//        talonConfiguration.peakOutputForward = 1.0;
//        talonConfiguration.peakOutputReverse = -1.0;
//
//        //talonConfiguration.velocityMeasurementPeriod = config.VELOCITY_MEASUREMENT_PERIOD;
//        talonConfiguration.velocityMeasurementWindow =
//            config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;
//
//        talonConfiguration.openloopRamp = config.OPEN_LOOP_RAMP_RATE;
//        talonConfiguration.closedloopRamp = config.CLOSED_LOOP_RAMP_RATE;
//
//        if (remoteSensorId >= 0) {
//            talonConfiguration.primaryPID.selectedFeedbackSensor =
//                FeedbackDevice.RemoteSensor0;
//            talonConfiguration.remoteFilter0.remoteSensorDeviceID = remoteSensorId;
//            talonConfiguration.remoteFilter0.remoteSensorSource =
//                RemoteSensorSource.CANCoder;
//        } else {
//            talonConfiguration.primaryPID.selectedFeedbackSensor =
//                isFalcon
//                    ? FeedbackDevice.IntegratedSensor
//                    : FeedbackDevice.CTRE_MagEncoder_Relative;
//        }
//        if (talonConfiguration instanceof TalonFXConfiguration) {
//            ((TalonFXConfiguration) talonConfiguration).supplyCurrLimit =
//                new SupplyCurrentLimitConfiguration(config.ENABLE_CURRENT_LIMIT, 0, 0, 0);
//        }
//
//        ErrorCode code = motor.configVelocityMeasurementPeriod(
//            config.VELOCITY_MEASUREMENT_PERIOD,
//            kTimeoutMs
//        );
//
//        talonConfiguration.clearPositionOnLimitF = false;
//        talonConfiguration.clearPositionOnLimitR = false;
//
//        talonConfiguration.enableOptimizations = true;
//
//        motor.configFactoryDefault(kTimeoutMs);
//
//        motor.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);
//
//        motor.setNeutralMode(config.NEUTRAL_MODE);
//        motor.selectProfileSlot(0, 0);
//
//        motor.setControlFramePeriod(
//            ControlFrame.Control_3_General,
//            config.CONTROL_FRAME_PERIOD_MS
//        );
//
//        motor.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_1_General,
//            config.GENERAL_STATUS_FRAME_RATE_MS,
//            kTimeoutMs
//        );
//        motor.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_2_Feedback0,
//            config.FEEDBACK_STATUS_FRAME_RATE_MS,
//            kTimeoutMs
//        );
//
//        motor.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_3_Quadrature,
//            config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
//            kTimeoutMs
//        );
//        motor.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_4_AinTempVbat,
//            config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
//            kTimeoutMs
//        );
//        motor.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_8_PulseWidth,
//            config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
//            kTimeoutMs
//        );
//
//        motor.configAllSettings(talonConfiguration, kTimeoutMs);
//        motor.setInverted(subsystem.invertMotor.contains(name));
//        motor.setSensorPhase(subsystem.invertSensorPhase.contains(name));
    }
}
