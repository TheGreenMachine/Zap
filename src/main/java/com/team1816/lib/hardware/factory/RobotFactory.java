package com.team1816.lib.hardware.factory;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.*;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.*;
import com.team1816.lib.hardware.components.gyro.GhostPigeonIMU;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.gyro.PigeonIMUImpl;
import com.team1816.lib.hardware.components.led.*;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazySparkMax;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.subsystems.drive.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import java.util.Objects;
import javax.annotation.Nonnull;

@Singleton
public class RobotFactory {

    private RobotConfiguration config;

    private enum PIDConfig {
        Azimuth,
        Drive,
        Generic,
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
                        .getResourceAsStream("yaml/" + robotName + ".config.yml")
                );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
    }

    public IGreenMotor getMotor(
        String subsystemName,
        String name,
        Map<String, PIDSlotConfiguration> pidConfigs,
        int remoteSensorId
    ) {
        IGreenMotor motor = null;
        var subsystem = getSubsystem(subsystemName);

        // Motor creation
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.talons, name)) {
                motor =
                    MotorFactory.createDefaultTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.infrastructure.canivoreBusName
                    );
            } else if (isHardwareValid(subsystem.falcons, name)) {
                motor =
                    MotorFactory.createDefaultTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.infrastructure.canivoreBusName
                    );
            } else if (isHardwareValid(subsystem.sparkmaxes, name)) {
                motor =
                    MotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name),
                        name,
                        subsystem,
                        pidConfigs
                    );
            }
            // Never make the victor a main
        }
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor =
                MotorFactory.createGhostMotor(
                    (int) (getConstant(subsystemName, "maxVelTicks100ms", 1, false)),
                    0,
                    name,
                    subsystem
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
        // Motor configuration
        if (subsystem.implemented && subsystem.invertMotor.contains(name)) {
            System.out.println("        Inverting " + name + " with ID " + motorId);
            motor.setInverted(true);
        }
        if (subsystem.implemented && subsystem.invertSensorPhase.contains(name)) {
            System.out.println(
                "       Inverting sensor phase of " + name + " with ID " + motorId
            );
            motor.setSensorPhase(true);
        }
        if (getConstant("configStatusFrames", 0) > 0) {
            setStatusFrame(motor); // make motor send one signal per second - FOR DEBUGGING!
        }
        return motor;
    }

    public IGreenMotor getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).pidConfig, -1); // not implemented for tank need to fix this
    }

    public IGreenMotor getFollowerMotor(
        String subsystemName,
        String name,
        IGreenMotor main
    ) {
        IGreenMotor followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && main != null) {
            if (isHardwareValid(subsystem.talons, name)) {
                // Talons must be following another Talon, cannot follow a Victor.
                followerMotor =
                    MotorFactory.createFollowerTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        main,
                        subsystem,
                        subsystem.pidConfig,
                        config.infrastructure.canivoreBusName
                    );
            } else if (isHardwareValid(subsystem.falcons, name)) {
                followerMotor =
                    MotorFactory.createFollowerTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        main,
                        subsystem,
                        subsystem.pidConfig,
                        config.infrastructure.canivoreBusName
                    );
            } else if (isHardwareValid(subsystem.sparkmaxes, name)) {
                followerMotor =
                    MotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name),
                        name,
                        subsystem
                    );
                ((LazySparkMax) followerMotor).follow(
                        main,
                        subsystem.invertMotor.contains(name)
                    );
                followerMotor.setInverted(main.getInverted());
            } else if (isHardwareValid(subsystem.victors, name)) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    MotorFactory.createFollowerVictor(
                        subsystem.victors.get(name),
                        name,
                        main
                    );
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                MotorFactory.createGhostMotor(
                    (int) getConstant(subsystemName, "maxVelTicks100ms"),
                    0,
                    name,
                    subsystem
                );
        }
        if (main != null) {
            followerMotor.setInverted(main.getInverted());
        }
        return followerMotor;
    }

    private boolean isHardwareValid(Map<String, Integer> map, String name) {
        if (map != null) {
            Integer hardwareId = map.get(name);
            return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
        }
        return false;
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

        var moduleConfig = new SwerveModule.ModuleConfig();
        moduleConfig.moduleName = name;
        moduleConfig.azimuthMotorName = module.azimuth; //getAzimuth and drive give ID i think - not the module name (ex: leftRear)
        moduleConfig.azimuthPid =
            getPidSlotConfig(subsystemName, "slot0", PIDConfig.Azimuth);
        moduleConfig.driveMotorName = module.drive;
        moduleConfig.drivePid = getPidSlotConfig(subsystemName, "slot0", PIDConfig.Drive);
        moduleConfig.azimuthEncoderHomeOffset = module.constants.get("encoderOffset");

        var canCoder = getCanCoder(subsystemName, name);

        return new SwerveModule(subsystemName, moduleConfig, canCoder);
    }

    public boolean hasCanCoder(String subsystemName, String name) {
        if (
            getSubsystem(subsystemName).swerveModules.modules.get(name) != null &&
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
                MotorFactory.createCanCoder(
                    subsystem.canCoders.get(module.canCoder),
                    subsystem.canCoders.get(subsystem.invertCanCoder) != null &&
                    subsystem.invertCanCoder.contains(module.canCoder)
                );
            if (getConstant("configStatusFrames") == 1) {
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
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.solenoids, name) && isPcmEnabled()) {
                return new SolenoidImpl(
                    config.infrastructure.pcmId,
                    config.infrastructure.pcmIsRev
                        ? PneumaticsModuleType.REVPH
                        : PneumaticsModuleType.CTREPCM,
                    subsystem.solenoids.get(name)
                );
            }
            reportGhostWarning("Solenoid", subsystemName, name);
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
                isHardwareValid(solenoidConfig.reverse) &&
                isPcmEnabled()
            ) {
                return new DoubleSolenoidImpl(
                    config.infrastructure.pcmId,
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
            if (getConstant("configStatusFrames") == 1) {
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
            candle =
                new CANdleImpl(subsystem.candle, config.infrastructure.canivoreBusName);
            candle.configFactoryDefault();
            candle.configStatusLedState(true);
            candle.configLOSBehavior(true);
            candle.configLEDType(CANdle.LEDStripType.BRG);
            candle.configBrightnessScalar(1);
            if (getConstant("configStatusFrames") == 1) {
                setStatusFrame(candle);
            }
            return candle;
        }
        reportGhostWarning(CANdle.class.getSimpleName(), subsystemName, "candle");
        return new GhostCANdle();
    }

    public ICompressor getCompressor() {
        if (isPcmEnabled()) {
            if (config.infrastructure.pcmIsRev) {
                return new CompressorImpl(getPcmId(), PneumaticsModuleType.REVPH);
            } else {
                return new CompressorImpl(getPcmId(), PneumaticsModuleType.CTREPCM);
            }
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId()); // root?
        return new GhostCompressor();
    }

    public Double getConstant(String name) {
        return getConstant(name, 0);
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
            DriverStation.reportWarning("Yaml constants:" + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public String getControlBoard() {
        return Objects.requireNonNullElse(config.controlboard, "empty");
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, true);
    }

    public double getConstant(
        String subsystemName,
        String name,
        double defaultVal,
        boolean showWarning
    ) {
        if (!getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (
            getSubsystem(subsystemName).constants == null ||
            !getSubsystem(subsystemName).constants.containsKey(name)
        ) {
            if (showWarning) {
                DriverStation.reportWarning(
                    "Yaml: subsystem \"" +
                    subsystemName +
                    "\" constant \"" +
                    name +
                    "\" missing",
                    defaultVal == 0
                );
            }
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName) {
        return getPidSlotConfig(subsystemName, "slot0", PIDConfig.Generic);
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

    public PowerDistribution getPd() {
        return new PowerDistribution(
            config.infrastructure.pdId,
            config.infrastructure.pdIsRev
                ? PowerDistribution.ModuleType.kRev
                : PowerDistribution.ModuleType.kCTRE
        );
    }

    public IPigeonIMU getPigeon() {
        int id = config.infrastructure.pigeonId;
        IPigeonIMU pigeon;
        if (!isHardwareValid(id)) {
            pigeon = new GhostPigeonIMU(id);
        } else if (config.infrastructure.isPigeon2) {
            System.out.println("Using Pigeon 2 for id: " + id);
            pigeon = new Pigeon2Impl(id, config.infrastructure.canivoreBusName);
        } else {
            System.out.println("Using old Pigeon for id: " + id);
            pigeon = new PigeonIMUImpl(id);
        }
        pigeon.configFactoryDefault();
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
        return pigeon;
    }

    public int getPcmId() {
        if (config.infrastructure.pcmId == null) return -1;
        return config.infrastructure.pcmId;
    }

    public boolean isPcmEnabled() {
        return getPcmId() > -1;
    }

    public boolean isCompressorEnabled() {
        return config.infrastructure.compressorEnabled;
    }

    private void reportGhostWarning(
        String type,
        String subsystemName,
        String componentName
    ) {
        System.out.println(
            "  " +
            type +
            " \"" +
            componentName +
            "\" invalid in Yaml for subsystem \"" +
            subsystemName +
            "\", using ghost!"
        );
    }

    private final int canMaxStatus = 100;

    private void setStatusFrame(IGreenMotor device) {
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
