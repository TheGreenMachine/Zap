package com.team1816.lib.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.team1816.lib.hardware.components.motor.GhostMotorControllerEnhanced;
import com.team1816.lib.hardware.components.motor.IConfigurableMotorController;
import com.team1816.lib.hardware.components.motor.LazyTalonFX;
import com.team1816.lib.hardware.components.motor.LazyTalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.*;

/**
 * A class to create Falcon (TalonFX), TalonSRX, VictorSPX, and GhostTalonSRX objects.
 * Based on FRC Team 254 The Cheesy Poof's 2018 TalonSRXFactory
 */
public class CtreMotorFactory {

    private static final int kTimeoutMs = RobotBase.isSimulation() ? 0 : 100;

    public static class Configuration {

        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
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

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically.  Potentially try to increase as much as possible.

        // Slave Config edits
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 255;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 255;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static IMotorControllerEnhanced createDefaultTalon(
        int id,
        String name,
        boolean isFalcon,
        SubsystemConfig subsystems,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        return createTalon(
            id,
            name,
            kDefaultConfiguration,
            isFalcon,
            subsystems,
            pidConfigList
        );
    }

    public static IMotorControllerEnhanced createPermanentSlaveTalon(
        int id,
        String name,
        boolean isFalcon,
        IMotorController master,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        final IMotorControllerEnhanced talon = createTalon(
            id,
            name,
            kSlaveConfiguration,
            isFalcon,
            subsystem,
            pidConfigList
        );
        System.out.println(
            "Slaving talon on " + id + " to talon on " + master.getDeviceID()
        );
        talon.follow(master);
        return talon;
    }

    private static IMotorControllerEnhanced createTalon(
        int id,
        String name,
        Configuration config,
        boolean isFalcon,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        IConfigurableMotorController talon = isFalcon
            ? new LazyTalonFX(id)
            : new LazyTalonSRX(id);
        configureMotorController(talon, name, config, isFalcon, subsystem, pidConfigList);

        return talon;
    }

    public static IMotorControllerEnhanced createGhostTalon() {
        return new GhostMotorControllerEnhanced();
    }

    public static IMotorController createDefaultVictor(int id) {
        return createVictor(id, kDefaultConfiguration);
    }

    public static IMotorController createPermanentSlaveVictor(
        int id,
        IMotorController master
    ) {
        final IMotorController victor = createVictor(id, kSlaveConfiguration);
        System.out.println(
            "Slaving victor on " + id + " to talon on " + master.getDeviceID()
        );
        victor.follow(master);
        return victor;
    }

    public static IMotorController createVictor(int id, Configuration config) {
        VictorSPX victor = new VictorSPX(id);
        //configureMotorController(victor, config);

        victor.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.Deactivated,
            LimitSwitchNormal.NormallyOpen,
            kTimeoutMs
        );

        victor.setStatusFramePeriod(
            StatusFrame.Status_1_General,
            config.GENERAL_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );
        victor.setStatusFramePeriod(
            StatusFrame.Status_2_Feedback0,
            config.FEEDBACK_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );
        victor.setStatusFramePeriod(
            StatusFrame.Status_4_AinTempVbat,
            config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );

        return victor;
    }

    private static SlotConfiguration toSlotConfiguration(PIDSlotConfiguration pidConfiguration){
        SlotConfiguration slotConfig = new SlotConfiguration();
        if(pidConfiguration!=null) {
            if(pidConfiguration.kP!=null)
                slotConfig.kP = pidConfiguration.kP;
            if(pidConfiguration.kI!=null)
                slotConfig.kI = pidConfiguration.kI;
            if(pidConfiguration.kD!=null)
                slotConfig.kD = pidConfiguration.kD;
            if(pidConfiguration.kP!=null)
                slotConfig.kF = pidConfiguration.kF;
            if(pidConfiguration.iZone!=null)
                slotConfig.integralZone = pidConfiguration.iZone;
            if(pidConfiguration.allowableError!=null)
                slotConfig.allowableClosedloopError = pidConfiguration.allowableError;
        }
        return slotConfig;
    }

    private static void configureMotorController(
        IConfigurableMotorController motor,
        String name,
        Configuration config,
        boolean isFalcon,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        BaseTalonConfiguration talonConfiguration;

        if (motor instanceof TalonFX) {
            talonConfiguration = new TalonFXConfiguration();
        } else if (motor instanceof TalonSRX) {
            talonConfiguration = new TalonSRXConfiguration();
        } else {
            return;
        }

        talonConfiguration.forwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
        talonConfiguration.forwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

        talonConfiguration.reverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
        talonConfiguration.reverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

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

        talonConfiguration.nominalOutputForward = 0;
        talonConfiguration.nominalOutputReverse = 0;
        talonConfiguration.neutralDeadband = config.NEUTRAL_DEADBAND;

        talonConfiguration.peakOutputForward = 1.0;
        talonConfiguration.peakOutputReverse = -1.0;

        //talonConfiguration.velocityMeasurementPeriod = config.VELOCITY_MEASUREMENT_PERIOD;
        talonConfiguration.velocityMeasurementWindow =
            config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

        talonConfiguration.openloopRamp = config.OPEN_LOOP_RAMP_RATE;
        talonConfiguration.closedloopRamp = config.CLOSED_LOOP_RAMP_RATE;

        talonConfiguration.primaryPID.selectedFeedbackSensor =
            isFalcon
                ? FeedbackDevice.IntegratedSensor
                : FeedbackDevice.CTRE_MagEncoder_Relative;

        if (talonConfiguration instanceof TalonFXConfiguration) {
            ((TalonFXConfiguration) talonConfiguration).supplyCurrLimit =
                new SupplyCurrentLimitConfiguration(config.ENABLE_CURRENT_LIMIT, 0, 0, 0);
        }

        ErrorCode code = motor.configVelocityMeasurementPeriod(
            config.VELOCITY_MEASUREMENT_PERIOD,
            kTimeoutMs
        );

        talonConfiguration.clearPositionOnLimitF = false;
        talonConfiguration.clearPositionOnLimitR = false;

        talonConfiguration.enableOptimizations = true;

        motor.configFactoryDefault(kTimeoutMs);

        motor.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        motor.setNeutralMode(config.NEUTRAL_MODE);
        motor.selectProfileSlot(0, 0);

        motor.setControlFramePeriod(
            ControlFrame.Control_3_General,
            config.CONTROL_FRAME_PERIOD_MS
        );

        motor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_1_General,
            config.GENERAL_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );
        motor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_2_Feedback0,
            config.FEEDBACK_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );

        motor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_3_Quadrature,
            config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );
        motor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_4_AinTempVbat,
            config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );
        motor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_8_PulseWidth,
            config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
            kTimeoutMs
        );

        motor.configAllSettings(talonConfiguration, kTimeoutMs);
        motor.setInverted(subsystem.invertMotor.contains(name));
        motor.setSensorPhase(subsystem.invertSensorPhase.contains(name));
    }

}
