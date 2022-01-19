package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.frcSeason.subsystems.SwerveModule;
import com.team1816.lib.hardware.components.CanifierImpl;
import com.team1816.lib.hardware.components.GhostCanifier;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import java.util.List;
import java.util.Map;
import javax.annotation.Nonnull;

public class RobotFactory {

    private RobotConfiguration config;
    private static boolean verbose;
    private static RobotFactory factory;

    public static RobotFactory getInstance() {
        if (factory == null) {
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) {
                robotName = "default";
                DriverStation.reportWarning(
                    "ROBOT_NAME environment variable not defined, falling back to default.config.yml!",
                    false
                );
            }
            factory = new RobotFactory(robotName);
        }
        return factory;
    }

    public RobotFactory(String configName) {
        System.out.println("Loading Config for " + configName);
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(configName + ".config.yml")
                );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
        verbose = getConstant("verbose") >= 1;
    }

    public IMotorControllerEnhanced getMotor(
        String subsystemName,
        String name,
        Map<String, PIDSlotConfiguration> pidConfigs
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
                        pidConfigs
                    );
            } else if (subsystem.falcons != null && isHardwareValid(subsystem.falcons.get(name))) {
                motor =
                    CtreMotorFactory.createDefaultTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        subsystem,
                        pidConfigs
                    );
            } // Never make the victor a master
        }
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor = CtreMotorFactory.createGhostTalon();
        }

        // Motor configuration
        if (subsystem.implemented && subsystem.invertMotor.contains(name)) {
            System.out.println("Inverting " + name + " with ID " + motor.getDeviceID());
            motor.setInverted(true);
        }
        if (subsystem.implemented && subsystem.invertSensorPhase.contains(name)) {
            System.out.println("Inverting sensor phase of " + name + " with ID " + motor.getDeviceID());
            motor.setSensorPhase(true);
        }

        return motor;
    }

    public IMotorControllerEnhanced getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).pidConfig);
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
            } else if (subsystem.falcons != null && isHardwareValid(subsystem.falcons.get(name))) {
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        master,
                        subsystem,
                        subsystem.pidConfig
                    );
            } else if (subsystem.victors != null && isHardwareValid(subsystem.victors.get(name))) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    CtreMotorFactory.createPermanentSlaveVictor(
                        subsystem.victors.get(name),
                        master
                    );
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning(
                "Motor",
                subsystemName,
                name
            );
            followerMotor = CtreMotorFactory.createGhostTalon();
        }
        if (master != null) {
            followerMotor.setInverted(master.getInverted());
        }
        return followerMotor;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1;
    }

    public SwerveModule getSwerveModule(
        String subsystemName,
        String name,
        Translation2d startPos
    ) {
        var subsystem = getSubsystem(subsystemName);
        SwerveModuleConfiguration module = subsystem.swerveModules.get(name);
        if (module == null) {
            DriverStation.reportError(
                "No swerve module with name " + name + " subsystem " + subsystemName,
                true
            );
            return null;
        }

        var swerveConstants = new SwerveModule.SwerveModuleConstants();
        swerveConstants.kName = name;
        swerveConstants.kAzimuthMotorName = module.modules.get(name).azimuth; //getAzimuth and drive give ID i think - not the module name (ex: leftRear)
        swerveConstants.kAzimuthPid = subsystem.swerveModules.get(name).azimuthPID.get(0);
        swerveConstants.kDriveMotorName = module.modules.get(name).drive;
        swerveConstants.kDrivePid = subsystem.swerveModules.get(name).drivePID.get(0);
        swerveConstants.kAzimuthEncoderHomeOffset = module.modules.get(name).constants.get("encoderOffset");
        swerveConstants.kInvertAzimuthSensorPhase = module.constants.get("invertedSensorPhase") == 1; //boolean

        return new SwerveModule(subsystemName, swerveConstants, startPos);
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.solenoids != null) {
            Integer solenoidId = subsystem.solenoids.get(name);
            if (subsystem.implemented && isHardwareValid(solenoidId) && isPcmEnabled()) {
                return new SolenoidImpl(config.pcm, PneumaticsModuleType.CTREPCM, solenoidId);
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
        if(config.subsystems.containsKey(subsystemName)) {
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
        if (!getSubsystem(subsystemName).implemented && getSubsystem(subsystemName).pidConfig!=null && getSubsystem(subsystemName).pidConfig.get(slot)!=null)
            return getSubsystem(subsystemName).pidConfig.get(slot);
        else { //default empty config
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
