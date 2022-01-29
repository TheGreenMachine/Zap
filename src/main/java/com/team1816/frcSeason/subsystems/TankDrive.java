package com.team1816.frcSeason.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.frcSeason.AutoModeSelector;
import com.team1816.frcSeason.Constants;
import com.team1816.frcSeason.planners.TankMotionPlanner;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.subsystems.DifferentialDrivetrain;
import com.team1816.lib.geometry.Pose2d;import com.team1816.lib.geometry.Pose2dWithCurvature;
import com.team1816.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

    private static TankDrive INSTANCE;
    private static final String NAME = "drivetrain";

    @Inject
    private static AutoModeSelector autoModeSelector;

    // hardware
    private final IMotorControllerEnhanced mLeftMaster, mRightMaster;
    private final IMotorController mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;

    // hardware states
    private BadLog mLogger;

    private final TankMotionPlanner mMotionPlanner;

    private double leftEncoderSimPosition = 0, rightEncoderSimPosition = 0;
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d;

    public TankDrive() {
        super();
        swerveModules = new SwerveModule[2];

        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster = factory.getMotor(NAME, "leftMain");
        mLeftSlaveA = factory.getMotor(NAME, "leftFollower", mLeftMaster);
        mLeftSlaveB = factory.getMotor(NAME, "leftFollowerTwo", mLeftMaster);
        mRightMaster = factory.getMotor(NAME, "rightMain");
        mRightSlaveA = factory.getMotor(NAME, "rightFollower", mRightMaster);
        mRightSlaveB = factory.getMotor(NAME, "rightFollowerTwo", mRightMaster);

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

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        if (((int) factory.getConstant(NAME, "pigeonOnTalon")) == 1) {
            var pigeonId = ((int) factory.getConstant(NAME, "pigeonId"));
            System.out.println("Pigeon on Talon " + pigeonId);
            IMotorController master = null;
            if (pigeonId == mLeftSlaveA.getDeviceID()) {
                master = mLeftSlaveA;
            } else if (pigeonId == mLeftSlaveB.getDeviceID()) {
                master = mLeftSlaveB;
            } else if (pigeonId == mRightSlaveA.getDeviceID()) {
                master = mRightSlaveA;
            } else if (pigeonId == mRightSlaveB.getDeviceID()) {
                master = mRightSlaveB;
            }
            if (master != null) {
                mPigeon = new PigeonIMU((TalonSRX) master);
            } else {
                mPigeon =
                    new PigeonIMU(
                        new TalonSRX((int) factory.getConstant(NAME, "pigeonId"))
                    );
            }
        } else {
            mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId"));
        }
        mPigeon.configFactoryDefault();

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        mMotionPlanner = new TankMotionPlanner();
    }

    @Override
    public double getDesiredHeading() {
        return getDesiredRotation2d().getDegrees();
    }

    public Rotation2d getDesiredRotation2d() {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return mPeriodicIO.path_setpoint.state().getRotation();
        }
        return mPeriodicIO.desired_heading;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            double leftAdjDemand = mPeriodicIO.left_demand;
            double rightAdjDemand = mPeriodicIO.right_demand;
            if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                leftAdjDemand = mPeriodicIO.left_demand * maxVelTicksPer100ms;
                rightAdjDemand = mPeriodicIO.right_demand * maxVelTicksPer100ms;
            }
            var driveTrainErrorPercent = .05;
            mPeriodicIO.left_error = leftAdjDemand * driveTrainErrorPercent;
            leftEncoderSimPosition +=
                (leftAdjDemand - mPeriodicIO.left_error) * tickRatioPerLoop;
            rightEncoderSimPosition += rightAdjDemand * tickRatioPerLoop;
            mPeriodicIO.left_position_ticks = leftEncoderSimPosition;
            mPeriodicIO.right_position_ticks = rightEncoderSimPosition;
            mPeriodicIO.left_velocity_ticks_per_100ms =
                leftAdjDemand - mPeriodicIO.left_error;
            mPeriodicIO.right_velocity_ticks_per_100ms = rightAdjDemand;
            // calculate rotation based on left/right vel differences
            gyroDrift -=
                (
                    mPeriodicIO.left_velocity_ticks_per_100ms -
                    mPeriodicIO.right_velocity_ticks_per_100ms
                ) /
                robotWidthTicks;
            mPeriodicIO.gyro_heading_no_offset =
                getDesiredRotation2d().rotateBy(Rotation2d.fromDegrees(gyroDrift));
            var rot2d = new edu.wpi.first.math.geometry.Rotation2d(
                mPeriodicIO.gyro_heading_no_offset.getRadians()
            );
            var xPos =
                Units.inches_to_meters(mRobotState.getEstimatedX()) +
                Constants.StartingPose.getTranslation().x();
            var yPos =
                Units.inches_to_meters(mRobotState.getEstimatedY()) +
                Constants.StartingPose.getTranslation().y();
            mRobotState.field.setRobotPose(xPos, yPos, rot2d);
        } else {
            mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
            mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
            mPeriodicIO.left_velocity_ticks_per_100ms =
                +mLeftMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.right_velocity_ticks_per_100ms =
                mRightMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.gyro_heading_no_offset =
                Rotation2d.fromDegrees(mPigeon.getFusedHeading());
        }
        mPeriodicIO.gyro_heading =
            mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mPeriodicIO.left_error = 0;
            mPeriodicIO.right_error = 0;
        } else {
            mPeriodicIO.left_error = mLeftMaster.getClosedLoopError(0);
            mPeriodicIO.right_error = mRightMaster.getClosedLoopError(0);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            if (isSlowMode) {
                mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand * 0.5);
                mRightMaster.set(
                    ControlMode.PercentOutput,
                    mPeriodicIO.right_demand * 0.5
                );
            } else {
                mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
                mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            }
        } else {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand);
        }
    }

    @Override
    protected void updateOpenLoopPeriodic(double timestamp) {
        // no openLoop update needed
    }

    /**
     * Configure talons for open loop control
     */

    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        setBrakeMode(signal.getBrakeMode());
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    @Override
    public void setOpenLoopRampRate(double openLoopRampRate) {
        super.setOpenLoopRampRate(openLoopRampRate);
        mLeftMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
        mRightMaster.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
    }

    @Override
    public void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean use_heading_controller
    ) {
        DriveSignal driveSignal = cheesyDriveHelper.cheesyDrive(forward, rotation, false); // quick turn temporarily eliminated
        // }

        if (getDriveControlState() == Drive.DriveControlState.TRAJECTORY_FOLLOWING) {
            if (
                driveSignal.getLeft() != 0 ||
                driveSignal.getRight() != 0 ||
                isDoneWithTrajectory()
            ) {
                setOpenLoop(driveSignal);
            }
        } else {
            setOpenLoop(driveSignal);
        }

        if (mDriveControlState != Drive.DriveControlState.OPEN_LOOP) {
            mDriveControlState = Drive.DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = driveSignal.getLeft();
        mPeriodicIO.right_demand = driveSignal.getRight();
    }

    @Override
    public void setVelocity(List<Translation2d> driveVectors) {
        setVelocity(
            new DriveSignal(driveVectors.get(0).norm(), driveVectors.get(1).norm()),
            DriveSignal.NEUTRAL
        );
    }

    /**
     * Configure talons for velocity control
     */
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

    public void setLogger(BadLog logger) {
        mLogger = logger;
    }

    @Override
    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            System.out.println("setBrakeMode " + on);
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    @Override
    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset =
            heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    @Override
    public void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory,
        Rotation2d targetHeading
    ) {
        if (mMotionPlanner != null) {
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
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
    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(
            getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR
        );
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (
            Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0
        );
    }

    public double getAngularVelocity() {
        return (
            (getRightLinearVelocity() - getLeftLinearVelocity()) /
            Constants.kDriveWheelTrackWidthInches
        );
    }

    public synchronized void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory
    ) {
        if (mMotionPlanner != null) {
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
    }

    @Override
    public boolean isDoneWithTrajectory() {
        if (
            mMotionPlanner == null ||
            mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING
        ) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    @Override
    protected void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            TankMotionPlanner.Output output = mMotionPlanner.update(
                timestamp,
                mRobotState.getFieldToVehicle(timestamp)
            );

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(
                    new DriveSignal(
                        radiansPerSecondToTicksPer100ms(output.left_velocity),
                        radiansPerSecondToTicksPer100ms(output.right_velocity)
                    ),
                    new DriveSignal(
                        output.left_feedforward_voltage / 12.0,
                        output.right_feedforward_voltage / 12.0
                    )
                );

                mPeriodicIO.left_accel =
                    radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel =
                    radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    @Override
    public void zeroSensors() {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(Rotation2d.identity());
        resetEncoders();
        if (mPigeon.getLastError() != ErrorCode.OK) {
            // BadLog.createValue("PigeonErrorDetected", "true");
            System.out.println(
                "Error detected with Pigeon IMU - check if the sensor is present and plugged in!"
            );
            System.out.println("Defaulting to drive straight mode");
            autoModeSelector.setHardwareFailure(true);
        } else {
            autoModeSelector.setHardwareFailure(false);
        }
    }

    @Override
    public void zeroSensors(Pose2d pose) {}

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        //        Timer.delay(3);

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

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(
        IMotorControllerEnhanced talon
    ) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
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

    public double getRightDriveTicks() {
        return mPeriodicIO.right_position_ticks;
    }

    public double getLeftDriveTicks() {
        return mPeriodicIO.left_position_ticks;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //        builder.addDoubleProperty(
        //            "Right Drive Distance",
        //            this::getRightEncoderDistance,
        //            null
        //        );
        //        builder.addDoubleProperty("Right Drive Ticks", this::getRightDriveTicks, null);
        //        builder.addDoubleProperty(
        //            "Left Drive Distance",
        //            this::getLeftEncoderDistance,
        //            null
        //        );
        //        builder.addDoubleProperty("Left Drive Ticks", this::getLeftDriveTicks, null);
        //        builder.addStringProperty(
        //            "Drive/ControlState",
        //            () -> this.getDriveControlState().toString(),
        //            null
        //        );
        //        builder.addBooleanProperty(
        //            "Drive/PigeonIMU State",
        //            () -> this.mPigeon.getLastError() == ErrorCode.OK,
        //            null
        //        );
        //
        //        SmartDashboard.putNumber("Drive/OpenLoopRampRate", this.openLoopRampRate);
        //        SmartDashboard
        //            .getEntry("Drive/OpenLoopRampRate")
        //            .addListener(
        //                notification -> {
        //                    setOpenLoopRampRate(notification.value.getDouble());
        //                },
        //                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        //            );
        //
        //        SmartDashboard.putBoolean("Drive/Zero Sensors", false);
        //        SmartDashboard
        //            .getEntry("Drive/Zero Sensors")
        //            .addListener(
        //                entryNotification -> {
        //                    if (entryNotification.value.getBoolean()) {
        //                        zeroSensors();
        //                        entryNotification.getEntry().setBoolean(false);
        //                    }
        //                },
        //                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        //            );
        // builder.addDoubleProperty("Drive/OpenLoopRampRateSetter", null, this::setOpenLoopRampRate);
        // builder.addDoubleProperty("Drive/OpenLoopRampRateValue", this::getOpenLoopRampRate, null);

        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        // SmartDashboard.putNumber("X Error", mPeriodicIO.error.getTranslation().x());
        // SmartDashboard.putNumber("Y error", mPeriodicIO.error.getTranslation().y());
        // SmartDashboard.putNumber("Theta Error", mPeriodicIO.error.getRotation().getDegrees());

        // SmartDashboard.putNumber("Left Voltage Kf", mPeriodicIO.left_voltage / getLeftLinearVelocity());
        // SmartDashboard.putNumber("Right Voltage Kf", mPeriodicIO.right_voltage / getRightLinearVelocity());

        // if (mPathFollower != null) {
        //     SmartDashboard.putNumber("Drive LTE", mPathFollower.getAlongTrackError());
        //     SmartDashboard.putNumber("Drive CTE", mPathFollower.getCrossTrackError());
        // } else {
        //     SmartDashboard.putNumber("Drive LTE", 0.0);
        //     SmartDashboard.putNumber("Drive CTE", 0.0);
        // }
    }
}
