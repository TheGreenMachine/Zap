package com.team1816.lib.subsystems;

import static com.team1816.lib.math.DriveConversions.metersPerSecondToTicksPer100ms;
import static com.team1816.lib.math.DriveConversions.rotationsToInches;

import com.ctre.phoenix.motorcontrol.*;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.math.DriveConversions;
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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    private static final String NAME = "drivetrain";

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Hardware
    private final IMotorControllerEnhanced mLeftMaster, mRightMaster;
    private final IMotorController mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;

    // Odometry
    private DifferentialDriveOdometry tankOdometry;
    private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d;

    // Control
    private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

    /**
     * Constructor
     */
    public TankDrive() {
        super(); // calls into the constructor of the abstract super class Drive, configures PeriodicIO and Pigeon (gyro) along with basic subsystem features
        // configure motors
        mLeftMaster = factory.getMotor(NAME, "leftMain");
        mLeftSlaveA = factory.getMotor(NAME, "leftFollower", mLeftMaster);
        mLeftSlaveB = factory.getMotor(NAME, "leftFollowerTwo", mLeftMaster);
        mRightMaster = factory.getMotor(NAME, "rightMain");
        mRightSlaveA = factory.getMotor(NAME, "rightFollower", mRightMaster);
        mRightSlaveB = factory.getMotor(NAME, "rightFollowerTwo", mRightMaster);

        // configure follower motor currentLimits
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            factory.getConstant(NAME, "currentLimit", 40),
            0,
            0
        );
        mLeftMaster.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        ((IMotorControllerEnhanced) mLeftSlaveA).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );
        ((IMotorControllerEnhanced) mLeftSlaveB).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );
        mRightMaster.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        ((IMotorControllerEnhanced) mRightSlaveA).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );
        ((IMotorControllerEnhanced) mRightSlaveB).configSupplyCurrentLimit(
                currentLimitConfig,
                Constants.kLongCANTimeoutMs
            );

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        tankOdometry = new DifferentialDriveOdometry(getHeading());
    }

    /**
     * Read/Write Periodics
     */

    @Override
    public synchronized void writeToHardware() { // sets the demands for hardware from the inputs provided
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            if (isSlowMode) {
                mLeftMaster.set(ControlMode.MotionMagic, mPeriodicIO.left_demand * 0.5);
                mRightMaster.set(
                    ControlMode.MotionMagic,
                    mPeriodicIO.right_demand * 0.5
                );
            } else {
                mLeftMaster.set(ControlMode.MotionMagic, mPeriodicIO.left_demand);
                mRightMaster.set(ControlMode.MotionMagic, mPeriodicIO.right_demand);
            }
        } else {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand);
        }
    }

    @Override
    public synchronized void readFromHardware() { // reads the actual output from robot and updates state
        mPeriodicIO.chassisSpeed =
            Constants.Tank.tankKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    getLeftVelocityActual(),
                    getRightVelocityActual()
                )
            );
        if (RobotBase.isSimulation()) {
            // this is needed because differentialDriveOdometry has no updatePoseWithTime equivalent hence it must be manually simulated
            double leftAdjDemand = mPeriodicIO.left_demand;
            double rightAdjDemand = mPeriodicIO.right_demand;
            if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                leftAdjDemand = mPeriodicIO.left_demand * maxVelTicksPer100ms;
                rightAdjDemand = mPeriodicIO.right_demand * maxVelTicksPer100ms;
            }
            // simulate lateral motion
            leftEncoderSimPosition += leftAdjDemand * tickRatioPerLoop;
            rightEncoderSimPosition += rightAdjDemand * tickRatioPerLoop;
            // simulate rotational motion
            mPeriodicIO.gyro_heading_no_offset =
                mPeriodicIO.gyro_heading_no_offset.rotateBy(
                    new Rotation2d(
                        mPeriodicIO.chassisSpeed.omegaRadiansPerSecond *
                        Constants.kLooperDt *
                        180 /
                        Math.PI *
                        0.01
                    )
                );
        } else {
            mPeriodicIO.gyro_heading_no_offset = Rotation2d.fromDegrees(mPigeon.getYaw());
        }
        mPeriodicIO.gyro_heading = mPeriodicIO.gyro_heading_no_offset;
        // the sole purpose of the error is for logging
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mPeriodicIO.left_error = 0;
            mPeriodicIO.right_error = 0;
        } else {
            mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
            mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);
        }
        tankOdometry.update(
            mPeriodicIO.gyro_heading,
            Units.inchesToMeters(getLeftEncoderDistance()),
            Units.inchesToMeters(getRightEncoderDistance())
        );
        updateRobotState();
    }

    /**
     * Config
     */

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");
        setBrakeMode(false);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mInfrastructure.getYaw());
        resetEncoders();
        resetOdometry(pose);
        mPeriodicIO.chassisSpeed = new ChassisSpeeds();
        autoModeSelector.setHardwareFailure(false);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        leftEncoderSimPosition = 0;
        rightEncoderSimPosition = 0;
    }

    @Override
    public Pose2d getPose() {
        return robotState.field_to_vehicle;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        tankOdometry.resetPosition(pose, getHeading());
        robotState.field_to_vehicle = pose;
    }

    @Override
    public void updateRobotState() {
        robotState.field_to_vehicle = tankOdometry.getPoseMeters();
        robotState.delta_vehicle =
            new ChassisSpeeds(
                mPeriodicIO.chassisSpeed.vxMetersPerSecond,
                mPeriodicIO.chassisSpeed.vyMetersPerSecond,
                mPeriodicIO.chassisSpeed.omegaRadiansPerSecond
            );
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean leftSide = EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(mLeftMaster),
            new EnhancedMotorChecker.NamedMotor("left_master", mLeftMaster)
        );
        boolean rightSide = EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(mRightMaster),
            new EnhancedMotorChecker.NamedMotor("right_master", mRightMaster)
        );

        boolean checkPigeon = mPigeon == null;

        System.out.println(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon) {
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        } else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    private void updateRobotPose() {
        robotState.field_to_vehicle = tankOdometry.getPoseMeters();
    }

    /**
     * Open loop control
     */

    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mPeriodicIO.left_feedforward = 0.0;
            mPeriodicIO.right_feedforward = 0.0;
        }
        setBrakeMode(signal.getBrakeMode());
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    @Override
    public void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean use_heading_controller
    ) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        DriveSignal driveSignal = cheesyDriveHelper.cheesyDrive(
            forward,
            rotation,
            false,
            false
        );

        setOpenLoop(driveSignal);
    }

    @Override
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        System.out.println("STARTING TRAJECTORY ");
        mPeriodicIO.timestamp = 0;
        mTrajectory = trajectory;
        mTrajectoryStart = 0;
        updateRobotState();
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        setBrakeMode(true);
        mOverrideTrajectory = false;
    }

    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to Velocity");
            mLeftMaster.selectProfileSlot(0, 0);
            mRightMaster.selectProfileSlot(0, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public void updateTrajectoryVelocities(Double leftVel, Double rightVel) {
        // Velocities are in m/sec comes from trajectory command
        var signal = new DriveSignal(
            metersPerSecondToTicksPer100ms(leftVel),
            metersPerSecondToTicksPer100ms(rightVel)
        );
        setVelocity(signal, DriveSignal.NEUTRAL);
    }

    /**
     * General getters and setters
     */

    @Override
    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            System.out.println("setBrakeMode " + on);
            mIsBrakeMode = on;

            mLeftMaster.set(ControlMode.Velocity, 0);
            mRightMaster.set(ControlMode.Velocity, 0);

            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public double getLeftEncoderRotations() { // this is used for nothing other than sim
        return leftEncoderSimPosition / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return rightEncoderSimPosition / DRIVE_ENCODER_PPR;
    }

    @Override
    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    @Override
    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return mLeftMaster.getSelectedSensorVelocity(0);
    }

    @Override
    public double getRightVelocityNativeUnits() {
        return mRightMaster.getSelectedSensorVelocity(0);
    }

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(
        IMotorControllerEnhanced talon
    ) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    public double getLeftVelocityActual() {
        double velocity = // might need to change
            DriveConversions.convertTicksToMeters(
                mLeftMaster.getSelectedSensorVelocity(0)
            ) *
            10;
        return velocity;
    }

    public double getRightVelocityActual() {
        double velocity = // might need to change
            DriveConversions.convertTicksToMeters(
                mRightMaster.getSelectedSensorVelocity(0)
            ) *
            10;
        return velocity;
    }

    @Override
    public double getLeftVelocityDemand() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            return mPeriodicIO.left_demand * maxVelTicksPer100ms;
        }
        return mPeriodicIO.left_demand;
    }

    @Override
    public double getRightVelocityDemand() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            return mPeriodicIO.right_demand * maxVelTicksPer100ms;
        }
        return mPeriodicIO.right_demand;
    }

    @Override
    public double getLeftVelocityError() {
        return mPeriodicIO.left_error;
    }

    @Override
    public double getRightVelocityError() {
        return mPeriodicIO.right_error;
    }

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        defaultPIDConfig.kI = 0.0;
        defaultPIDConfig.kD = 0.0;
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory.getSubsystem(NAME).pidConfig.getOrDefault(pidSlot, defaultPIDConfig)
            : defaultPIDConfig;
    }
}
