package com.team1816.lib.hardware;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.sensors.*;
import com.team1816.lib.hardware.components.*;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import javax.annotation.Nonnull;

public class RobotFactory {

    private RobotConfiguration config;
    private static boolean verbose;
    private static RobotFactory factory;

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
                        remoteSensorId
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
                        remoteSensorId
                    );
            } // Never make the victor a master
        }
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor =
                CtreMotorFactory.createGhostTalon(
                    config.constants.get("maxTicks").intValue()
                );
        }

        var motorId = motor.getDeviceID();

        //no need to invert of print if ghosted
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
                        subsystem.pidConfig
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
                        subsystem.pidConfig
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
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                CtreMotorFactory.createGhostTalon(
                    config.constants.get("maxTicks").intValue()
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
        swerveConstants.kAzimuthPid = subsystem.swerveModules.azimuthPID.get("slot0");
        swerveConstants.kDriveMotorName = module.drive;
        swerveConstants.kDrivePid = subsystem.swerveModules.drivePID.get("slot0");
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
                    PneumaticsModuleType.CTREPCM,
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
                    PneumaticsModuleType.CTREPCM,
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
        if (subsystem.implemented && isHardwareValid(subsystem.canifier)) {
            return new CanifierImpl(subsystem.canifier);
        }
        reportGhostWarning("CANifier", subsystemName, "canifier");
        return new GhostCanifier();
    }

    public CANdle getCandle(String subsystemName, int defaultVal) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && isHardwareValid((subsystem.candle))) {
            return new CANdle(subsystem.candle);
        } else if (defaultVal > -1) {
            return new CANdle(defaultVal);
        } else {
            //ghost
        }
        return null;
    }

    public ICompressor getCompressor() {
        if (isPcmEnabled()) {
            return new CompressorImpl(getPcmId(), PneumaticsModuleType.CTREPCM);
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId());
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

    public Double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportError("Yaml constants:" + name + " missing", false);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        if (getConstants() == null || !getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (!getSubsystem(subsystemName).constants.containsKey(name)) {
            DriverStation.reportError(
                "Yaml " + subsystemName + " constants:" + name + " missing",
                false
            );
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName, String slot) {
        var subsystem = getSubsystem(subsystemName);
        if (
            subsystem.implemented &&
            subsystem.pidConfig != null &&
            subsystem.pidConfig.get(slot) != null
        ) return subsystem.pidConfig.get(slot); else { //default empty config
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

    public IPigeonIMU getPigeon(int id) {
        if (id < 0) {
            return new GhostPigeonIMU(id);
        } else {
            return new PigeonIMUImpl(id);
        }
    }

    public IPigeonIMU getPigeon(IMotorControllerEnhanced motor) {
        if ((int) factory.getConstant(Drive.NAME, "pigeonId", -1) < 0) {
            return new GhostPigeonIMU(motor);
        } else {
            return new PigeonIMUImpl(motor);
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
}
