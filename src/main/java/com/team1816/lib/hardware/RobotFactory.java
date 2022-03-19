package com.team1816.lib.hardware;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.*;
import com.team1816.lib.hardware.components.*;
import com.team1816.lib.hardware.components.motor.LazySparkMax;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.math.DriveConversions;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import javax.annotation.Nonnull;

import static com.team1816.lib.subsystems.Subsystem.factory;

public class RobotFactory {

    private RobotConfiguration config;
    private static boolean verbose;
    private static RobotFactory factory;

    private enum PIDConfig {
        Azimuth,
        Drive,
        Generic,
    }

    public static RobotFactory getInstance() {
        if (factory == null) {
            factory = new RobotFactory();
        }
        return factory;
    }

    public RobotFactory() {
        var robotName = System.getenv("ROBOT_NAME");
        if (robotName == null) {
            robotName = "default";
            DriverStation.reportWarning(
                "ROBOT_NAME environment variable not defined, falling back to default.config.yml!",
                false
            );
        }
        System.out.println("Loading Config for " + robotName);
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(robotName + ".config.yml")
                );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
        verbose = getConstant("verbose") >= 1;
    }

    public IMotorControllerEnhanced getMotor(
        String subsystemName,
        String name,
        Map<String, PIDSlotConfiguration> pidConfigs,
        int remoteSensorId
    ) {
        IMotorControllerEnhanced motor = null;
        var subsystem = getSubsystem(subsystemName);

        // Motor creation
        if (subsystem.implemented) {
            if (subsystem.talons != null && isHardwareValid(subsystem.talons.get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.canivoreBusName
                    );
            } else if (
                subsystem.falcons != null && isHardwareValid(subsystem.falcons.get(name))
            ) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.canivoreBusName
                    );
            } else if (
                subsystem.sparkmaxes != null &&
                isHardwareValid(subsystem.sparkmaxes.get(name))
            ) {
                motor =
                    RevMotorFactory.createDefaultSpark(
                        subsystem.sparkmaxes.get(name),
                        name,
                        subsystem,
                        pidConfigs,
                        remoteSensorId
                    );
            }
            // Never make the victor a master
        }
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor =
                CtreMotorFactory.createGhostTalon(
                    //                    config.constants.get("maxTicks").intValue()
                    (int) (
                        DriveConversions.inchesPerSecondToTicksPer100ms(
                            Constants.kOpenLoopMaxVelMeters / 0.0254 // this may not work if 2 diff velocities are used depending on if in auto or not
                        )
                    ),
                    0
                );
        } else {
            System.out.println(
                "Created " +
                motor.getClass().getSimpleName() +
                " id:" +
                motor.getDeviceID()
            );
        }

        var motorId = motor.getDeviceID();

        //no need to invert of print if ghosted - this is done in both here and CTREMotorFactory - why?
        if (motorId >= 0) {
            // Motor configuration
            if (subsystem.implemented && subsystem.invertMotor.contains(name)) {
                System.out.println("Inverting " + name + " with ID " + motorId);
                motor.setInverted(true);
            }
            if (subsystem.implemented && subsystem.invertSensorPhase.contains(name)) {
                System.out.println(
                    "Inverting sensor phase of " + name + " with ID " + motorId
                );
                motor.setSensorPhase(true);
            }
        }
        if (factory.getConstant("configStatusFrames", 0) > 0) {
            setStatusFrame(motor); // make motor send one signal per second - FOR DEBUGGING!
        }
        return motor;
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).pidConfig, -1); // not implemented for tank need to fix this
    }

    public IMotorController getMotor(
        String subsystemName,
        String name,
        IMotorController master
    ) { // TODO: optimize this method
        IMotorController followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && master != null) {
            if (subsystem.talons != null && isHardwareValid(subsystem.talons.get(name))) {
                // Talons must be following another Talon, cannot follow a Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        master,
                        subsystem,
                        subsystem.pidConfig,
                        config.canivoreBusName
                    );
            } else if (
                subsystem.falcons != null && isHardwareValid(subsystem.falcons.get(name))
            ) {
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        master,
                        subsystem,
                        subsystem.pidConfig,
                        config.canivoreBusName
                    );
            } else if (
                subsystem.victors != null && isHardwareValid(subsystem.victors.get(name))
            ) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveVictor(
                        subsystem.victors.get(name),
                        master
                    );
            } else if (
                subsystem.sparkmaxes != null && isHardwareValid(subsystem.sparkmaxes.get(name))
            ) {
                followerMotor =
                    RevMotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name)
                    );
                followerMotor.follow(master);
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                CtreMotorFactory.createGhostTalon(
                    (int) factory.getConstant(subsystemName, "maxTicks"),
                    0
                );
        }
        if (master != null) {
            followerMotor.setInverted(master.getInverted());
        }
        return followerMotor;
    }

    public IMotorController getMotor( // a hack to circumnavigate sparkMax follower methods
        String subsystemName,
        String name,
        IMotorController master,
        boolean invert
    ) { // TODO: optimize this method
        IMotorController followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.sparkmaxes != null && isHardwareValid(subsystem.sparkmaxes.get(name)))
        {
                followerMotor =
                    RevMotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name)
                    );
            ((LazySparkMax) followerMotor).follow(master, invert);
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                CtreMotorFactory.createGhostTalon(
                    (int) factory.getConstant(subsystemName, "maxTicks"),
                    0
                );
        }
        if (master != null) {
            followerMotor.setInverted(master.getInverted());
        }
        return followerMotor;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
    }

    public SwerveModule getSwerveModule(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        ModuleConfiguration module = subsystem.swerveModules.modules.get(name);
        if (module == null) {
            DriverStation.reportError(
                "No swerve module with name " + name + " subsystem " + subsystemName,
                true
            );
            return null;
        }

        var swerveConstants = new Constants.Swerve();
        swerveConstants.kName = name;
        swerveConstants.kAzimuthMotorName = module.azimuth; //getAzimuth and drive give ID i think - not the module name (ex: leftRear)
        swerveConstants.kAzimuthPid =
            getPidSlotConfig(subsystemName, "slot0", PIDConfig.Azimuth);
        swerveConstants.kDriveMotorName = module.drive;
        swerveConstants.kDrivePid =
            getPidSlotConfig(subsystemName, "slot0", PIDConfig.Drive);
        swerveConstants.kAzimuthEncoderHomeOffset = module.constants.get("encoderOffset");
        swerveConstants.kInvertAzimuthSensorPhase =
            (module.constants.get("invertedSensorPhase") != null) &&
            (module.constants.get("invertedSensorPhase") == 1); //boolean

        var canCoder = getCanCoder(subsystemName, name);

        var swerveModule = new SwerveModule(subsystemName, swerveConstants, canCoder);
        return swerveModule;
    }

    public boolean hasCanCoder(String subsystemName, String name) {
        if (
            getSubsystem(subsystemName).swerveModules.modules.get(name).canCoder !=
            null &&
            getSubsystem(subsystemName).swerveModules.modules.get(name).canCoder != null
        ) {
            return true;
        }
        return false;
    }

    public CANCoder getCanCoder(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        var module = subsystem.swerveModules.modules.get(name);
        CANCoder canCoder = null;
        if (
            hasCanCoder(subsystemName, name) &&
            subsystem.canCoders.get(module.canCoder) >= 0
        ) {
            canCoder =
                CtreMotorFactory.createCanCoder(
                    subsystem.canCoders.get(module.canCoder),
                    subsystem.canCoders.get(subsystem.invertCanCoder) != null &&
                    subsystem.invertCanCoder.contains(module.canCoder)
                ); //TODO: For now placeholder true is placed
            if (factory.getConstant("configStatusFrames") == 1) {
                setStatusFrame(canCoder); // make canCoder send one signal per second - FOR DEBUGGING!
            }
        } else {
            // ghost. potentially implement this in the future
        }

        return canCoder;
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.solenoids != null) {
            Integer solenoidId = subsystem.solenoids.get(name);
            if (subsystem.implemented && isHardwareValid(solenoidId) && isPcmEnabled()) {
                return new SolenoidImpl(
                    config.pcm,
                    factory.getConstant("phIsRev") > 0
                        ? PneumaticsModuleType.REVPH
                        : PneumaticsModuleType.CTREPCM,
                    solenoidId
                );
            }
            if (subsystem.implemented) {
                reportGhostWarning("Solenoid", subsystemName, name);
            }
            return new GhostSolenoid();
        }
        return new GhostSolenoid();
    }

    @Nonnull
    public IDoubleSolenoid getDoubleSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (getSubsystem(subsystemName).doubleSolenoids != null) {
            DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystemName)
                .doubleSolenoids.get(name);
            if (
                subsystem.implemented &&
                solenoidConfig != null &&
                isHardwareValid(solenoidConfig.forward) &&
                isHardwareValid(solenoidConfig.forward) &&
                isPcmEnabled()
            ) {
                return new DoubleSolenoidImpl(
                    config.pcm,
                    PneumaticsModuleType.REVPH,
                    solenoidConfig.forward,
                    solenoidConfig.reverse
                );
            }
        }
        reportGhostWarning("DoubleSolenoid", subsystemName, name);
        return new GhostDoubleSolenoid();
    }

    @Nonnull
    public ICanifier getCanifier(String subsystemName) {
        var subsystem = getSubsystem(subsystemName);
        ICanifier canifier;
        if (subsystem.implemented && isHardwareValid(subsystem.canifier)) {
            canifier = new CanifierImpl(subsystem.canifier);
            if (factory.getConstant("configStatusFrames") == 1) {
                setStatusFrame((CANifier) canifier); // make signal time of 1 sec
            }
            return canifier;
        }
        reportGhostWarning("CANifier", subsystemName, "canifier");

        return new GhostCanifier();
    }

    public ICANdle getCandle(String subsystemName) {
        ICANdle candle;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && isHardwareValid((subsystem.candle))) {
            candle = new CANdleImpl(subsystem.candle, config.canivoreBusName);
            candle.configFactoryDefault();
            candle.configStatusLedState(true);
            candle.configLOSBehavior(true);
            candle.configLEDType(CANdle.LEDStripType.BRG);
            candle.configBrightnessScalar(1);
            if (factory.getConstant("configStatusFrames") == 1) {
                setStatusFrame(candle);
            }
            return candle;
        }
        reportGhostWarning(CANdle.class.getSimpleName(), subsystemName, "candle");
        return new GhostCANdle();
    }

    public ICompressor getCompressor(boolean isREV) {
        if (isPcmEnabled()) {
            if (isREV){
                return new CompressorImpl(
                    getPcmId(),
                    PneumaticsModuleType.REVPH
                );
            } else {
                return new CompressorImpl(getPcmId(), PneumaticsModuleType.CTREPCM);
            }
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId()); // root?
        return new GhostCompressor();
    }

    public Double getConstant(String name) {
        return getConstant(name, 0.0);
    }

    public Map<String, Double> getConstants() {
        return config.constants;
    }

    public SubsystemConfig getSubsystem(String subsystemName) {
        if (config.subsystems.containsKey(subsystemName)) {
            var subsystem = config.subsystems.get(subsystemName);
            if (subsystem == null) {
                subsystem = new SubsystemConfig();
                subsystem.implemented = false;
                System.out.println("Subsystem not defined: " + subsystemName);
            }
            return subsystem;
        }
        SubsystemConfig subsystem = new SubsystemConfig();
        subsystem.implemented = false;
        return subsystem;
    }

    public double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportError("Yaml constants:" + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        if (!getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (
            getSubsystem(subsystemName).constants == null ||
            !getSubsystem(subsystemName).constants.containsKey(name)
        ) {
            DriverStation.reportError(
                "Yaml " + subsystemName + " constants:" + name + " missing",
                defaultVal == 0
            );
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName, String slot) {
        return getPidSlotConfig(subsystemName, slot, PIDConfig.Generic);
    }

    public PIDSlotConfiguration getPidSlotConfig(
        String subsystemName,
        String slot,
        PIDConfig configType
    ) {
        var subsystem = getSubsystem(subsystemName);
        Map<String, PIDSlotConfiguration> config = null;
        if (subsystem.implemented) {
            switch (configType) {
                case Azimuth:
                    config = subsystem.swerveModules.azimuthPID;
                    break;
                case Drive:
                    config = subsystem.swerveModules.drivePID;
                    break;
                case Generic:
                    config = subsystem.pidConfig;
                    break;
            }
        }
        if (config != null && config.get(slot) != null) return config.get(slot); else {
            if (subsystem.implemented) {
                DriverStation.reportError(
                    "pidConfig missing for " + subsystemName + " " + slot,
                    true
                );
                return null;
            } else {
                // return a default config if not implemented
                PIDSlotConfiguration pidSlotConfiguration = new PIDSlotConfiguration();
                pidSlotConfiguration.kP = 0.0;
                pidSlotConfiguration.kI = 0.0;
                pidSlotConfiguration.kD = 0.0;
                pidSlotConfiguration.kF = 0.0;
                pidSlotConfiguration.iZone = 0;
                pidSlotConfiguration.allowableError = 0.0;
                return pidSlotConfiguration;
            }
        }
    }

    public IPigeonIMU getPigeon() {
        int id = (int) factory.getConstant("pigeonId", -1);
        IPigeonIMU pigeonIMU;
        if (!isHardwareValid(id)) {
            return new GhostPigeonIMU(id);
        } else if (factory.getConstant("isPigeon2") > 0) {
            System.out.println("Using Pigeon 2 for id: " + id);
            pigeonIMU = new Pigeon2Impl(id, config.canivoreBusName);
            return pigeonIMU;
        } else {
            System.out.println("Using old Pigeon for id: " + id);
            pigeonIMU = new PigeonIMUImpl(id);
            return pigeonIMU;
        }
    }

    public int getPcmId() {
        if (config.pcm == null) return -1;
        return config.pcm;
    }

    public boolean isPcmEnabled() {
        return getPcmId() > -1;
    }

    public static boolean isVerbose() {
        return verbose;
    }

    private void reportGhostWarning(
        String type,
        String subsystemName,
        String componentName
    ) {
        System.out.println(
            "  " +
            type +
            "  " +
            componentName +
            " not defined or invalid in config for subsystem " +
            subsystemName +
            ", using ghost!"
        );
    }

    private final int canMaxStatus = 100;

    private void setStatusFrame(IMotorControllerEnhanced device) {
        device.setStatusFramePeriod(StatusFrame.Status_1_General, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_6_Misc, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, canMaxStatus, 100);
        device.setStatusFramePeriod(
            StatusFrame.Status_9_MotProfBuffer,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(StatusFrame.Status_10_Targets, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, canMaxStatus, 100);
        device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, canMaxStatus, 100);
        device.setStatusFramePeriod(
            StatusFrame.Status_15_FirmwareApiStatus,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, canMaxStatus, 100);
    }

    private void setStatusFrame(BasePigeon device) {
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 100);
        device.setStatusFramePeriod(
            PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass,
            100
        );
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 100);
        device.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 100);
    }

    private void setStatusFrame(CANCoder device) {
        //        device.setStatusFramePeriod(CANCoderStatusFrame.SensorData, canMaxStatus, 100);
        device.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, canMaxStatus, 100);
    }

    private void setStatusFrame(CANifier device) {
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_1_General,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_2_General,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_3_PwmInputs0,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_4_PwmInputs1,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_5_PwmInputs2,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANifierStatusFrame.Status_6_PwmInputs3,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(CANifierStatusFrame.Status_8_Misc, canMaxStatus, 100);
    }

    private void setStatusFrame(ICANdle device) {
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_1_General,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels,
            canMaxStatus,
            100
        );
        device.setStatusFramePeriod(
            CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels,
            canMaxStatus,
            100
        );
    }
}
