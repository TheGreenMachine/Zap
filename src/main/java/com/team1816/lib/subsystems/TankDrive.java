package com.team1816.lib.subsystems;

import static com.team1816.lib.math.DriveConversions.*;

import com.ctre.phoenix.motorcontrol.*;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.season.Constants;
import com.team1816.season.auto.AutoModeSelector;
import com.team1816.season.subsystems.LedManager;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    private static final String NAME = "drivetrain";

    // Components
    private final IGreenMotor leftMain, rightMain;
    private final IGreenMotor leftFollowerA, rightFollowerA, leftFollowerB, rightFollowerB;

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Odometry
    private DifferentialDriveOdometry tankOdometry;
    private final CheesyDriveHelper driveHelper = new CheesyDriveHelper();
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d; // Convert Ticks/100MS into Ticks/Robot Loop

    // States
    public double leftPowerDemand, rightPowerDemand; // % Output (-1 to 1) - used in OPEN_LOOP
    public double leftVelDemand, rightVelDemand; // Velocity (Ticks/100MS) - used in TRAJECTORY_FOLLOWING

    private double leftActualDistance = 0, rightActualDistance = 0; // Meters

    private double leftActualVelocity, rightActualVelocity; // Ticks/100MS

    double leftErrorClosedLoop;
    double rightErrorClosedLoop;

    /**
     * Constructor
     */
    public TankDrive() {
        super();
        // configure motors
        leftMain = factory.getMotor(NAME, "leftMain");
        leftFollowerA = factory.getMotor(NAME, "leftFollower", leftMain);
        leftFollowerB = factory.getMotor(NAME, "leftFollowerTwo", leftMain);
        rightMain = factory.getMotor(NAME, "rightMain");
        rightFollowerA = factory.getMotor(NAME, "rightFollower", rightMain);
        rightFollowerB = factory.getMotor(NAME, "rightFollowerTwo", rightMain);

        // configure follower motor currentLimits
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            factory.getConstant(NAME, "currentLimit", 40),
            0,
            0
        );
        leftMain.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        leftFollowerA.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        leftFollowerB.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightMain.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightFollowerA.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightFollowerB.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );

        setOpenLoop(DriveSignal.NEUTRAL);

        tankOdometry = new DifferentialDriveOdometry(getActualHeading());
    }

    /**
     * Read/Write Periodics
     */

    @Override
    public synchronized void writeToHardware() { // sets the demands for hardware from the inputs provided
        if (controlState == ControlState.OPEN_LOOP) {
            leftMain.set(
                ControlMode.PercentOutput,
                isSlowMode ? (leftPowerDemand * 0.5) : leftPowerDemand
            );
            rightMain.set(
                ControlMode.PercentOutput,
                isSlowMode ? (rightPowerDemand * 0.5) : rightPowerDemand
            );
        } else {
            leftMain.set(ControlMode.Velocity, leftVelDemand);
            rightMain.set(ControlMode.Velocity, rightVelDemand);
        }
    }

    @Override
    public synchronized void readFromHardware() {
        // update current motor velocities and distance traveled
        leftActualVelocity = leftMain.getSelectedSensorVelocity(0);
        rightActualVelocity = rightMain.getSelectedSensorVelocity(0);
        leftActualDistance += ticksToMeters(leftActualVelocity * tickRatioPerLoop);
        rightActualDistance += ticksToMeters(rightActualVelocity * tickRatioPerLoop);

        // update error (only if in closed loop where knowing it is useful)
        if (controlState == ControlState.TRAJECTORY_FOLLOWING) {
            leftErrorClosedLoop = leftMain.getClosedLoopError(0);
            rightErrorClosedLoop = rightMain.getClosedLoopError(0);
        }

        // update current movement of the whole drivetrain
        chassisSpeed =
            Constants.Tank.tankKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(getLeftMPSActual(), getRightMPSActual())
            );

        // update actual heading from gyro (pigeon)
        if (RobotBase.isSimulation()) {
            simulateGyroOffset();
        }
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());

        // send updated information to robotState and odometry calculator
        tankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        updateRobotState();
    }

    /**
     * Config
     */

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");

        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());
        resetEncoders();
        resetOdometry(pose);

        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(
            controlState == ControlState.OPEN_LOOP
                ? DriveSignal.NEUTRAL
                : DriveSignal.BRAKE
        );
        setBraking(controlState == ControlState.TRAJECTORY_FOLLOWING);
    }

    public synchronized void resetEncoders() {
        leftMain.setSelectedSensorPosition(0, 0, 0);
        rightMain.setSelectedSensorPosition(0, 0, 0);
        leftActualDistance = 0;
        rightActualDistance = 0;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        tankOdometry.resetPosition(pose, getActualHeading());
        tankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        updateRobotState();
    }

    @Override
    public void updateRobotState() {
        robotState.fieldToVehicle = tankOdometry.getPoseMeters();
        robotState.deltaVehicle =
            new ChassisSpeeds(
                chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond,
                chassisSpeed.omegaRadiansPerSecond
            );
    }

    /**
     * Open loop control
     */

    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            controlState = ControlState.OPEN_LOOP;
            leftErrorClosedLoop = 0;
            rightErrorClosedLoop = 0;
        }
        leftPowerDemand = signal.getLeft();
        rightPowerDemand = signal.getRight();

        leftVelDemand = leftPowerDemand * maxVelTicks100ms;
        rightVelDemand = rightPowerDemand * maxVelTicks100ms;
    }

    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        DriveSignal driveSignal = driveHelper.cheesyDrive(
            forward,
            rotation,
            false,
            false
        );

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(driveSignal);
        }
    }

    public synchronized void setVelocity(DriveSignal signal) {
        if (controlState == ControlState.OPEN_LOOP) {
            System.out.println("Switching to Velocity");

            leftMain.selectProfileSlot(0, 0);
            rightMain.selectProfileSlot(0, 0);

            leftMain.configNeutralDeadband(0.0, 0);
            rightMain.configNeutralDeadband(0.0, 0);
        }

        leftVelDemand = signal.getLeft();
        rightVelDemand = signal.getRight();
    }

    public void updateTrajectoryVelocities(Double leftVel, Double rightVel) {
        // Velocities are in m/sec comes from trajectory command
        var signal = new DriveSignal(
            metersPerSecondToTicksPer100ms(leftVel),
            metersPerSecondToTicksPer100ms(rightVel)
        );
        setVelocity(signal);
    }

    /**
     * General getters and setters
     */

    @Override
    public synchronized void setBraking(boolean braking) {
        if (isBraking != braking) {
            System.out.println("braking: " + braking);
            isBraking = braking;

            if (braking) {
                leftMain.set(ControlMode.Velocity, 0);
                rightMain.set(ControlMode.Velocity, 0);
            }
            // TODO ensure that changing neutral modes won't backfire while we're using brushless motors
            NeutralMode mode = braking ? NeutralMode.Brake : NeutralMode.Coast;

            rightMain.setNeutralMode(mode);
            rightFollowerA.setNeutralMode(mode);
            rightFollowerB.setNeutralMode(mode);

            leftMain.setNeutralMode(mode);
            leftFollowerA.setNeutralMode(mode);
            leftFollowerB.setNeutralMode(mode);
        }
    }

    public double getLeftMPSActual() {
        return ticksPer100MSToMPS(leftActualVelocity);
    }

    public double getRightMPSActual() {
        return ticksPer100MSToMPS(rightActualVelocity);
    }

    @Override
    public double getLeftVelocityTicksDemand() {
        return leftVelDemand;
    }

    @Override
    public double getRightVelocityTicksDemand() {
        return rightVelDemand;
    }

    @Override
    public double getLeftVelocityTicksActual() {
        return leftMain.getSelectedSensorVelocity(0);
    }

    @Override
    public double getRightVelocityTicksActual() {
        return rightMain.getSelectedSensorVelocity(0);
    }

    @Override
    public double getLeftDistance() {
        return leftActualDistance;
    }

    @Override
    public double getRightDistance() {
        return rightActualDistance;
    }

    @Override
    public double getLeftError() {
        return leftErrorClosedLoop;
    }

    @Override
    public double getRightError() {
        return rightErrorClosedLoop;
    }

    @Override
    public boolean checkSystem() {
        boolean leftSide = EnhancedMotorChecker.checkMotor(this, leftMain);
        boolean rightSide = EnhancedMotorChecker.checkMotor(this, rightMain);

        boolean checkPigeon = infrastructure.getPigeon() == null;

        System.out.println(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon) {
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        } else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        defaultPIDConfig.kI = 0.0;
        defaultPIDConfig.kD = 0.0;
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory.getSubsystem(NAME).pidConfig.getOrDefault("slot0", defaultPIDConfig)
            : defaultPIDConfig;
    }
}
