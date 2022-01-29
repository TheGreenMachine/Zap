package com.team1816.frcSeason.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.frcSeason.Constants;
import com.team1816.frcSeason.RobotState;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.TrackableDrivetrain;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.*;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public abstract class Drive
    extends Subsystem
    implements TrackableDrivetrain, PidProvider {

    public interface Factory {
        Drive getInstance();
    }

    public static final String NAME = "drivetrain";

    // Components
    @Inject
    protected static LedManager ledManager;

    protected PigeonIMU mPigeon;
    protected SwerveModule[] swerveModules;

    // control states
    protected DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    @Inject
    protected static RobotState mRobotState;

    // Odometry variables
    protected Pose2d pose = Pose2d.identity();
    protected Pose2d startingPosition = Pose2d.identity();
    protected double lastUpdateTimestamp = 0;

    // hardware states
    protected String pidSlot = "slot0";
    protected boolean mIsBrakeMode;
    protected Rotation2d mGyroOffset = Rotation2d.identity();
    protected double openLoopRampRate;

    protected PeriodicIO mPeriodicIO;
    protected boolean mOverrideTrajectory = false;

    protected boolean isSlowMode;
    protected double rotationScalar = 1;
    protected boolean robotCentric = false;
    protected SendableChooser<DriveHelper> driveHelperChooser;

    // Simulator
    protected double gyroDrift;
    protected final double robotWidthTicks =
        inchesPerSecondToTicksPer100ms(Constants.kDriveWheelTrackWidthInches) * Math.PI;

    // Constants
    public static final double maxVelTicksPer100ms = factory.getConstant("maxTicks");
    public static final double DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
    public static final List<Translation2d> ZERO_DRIVE_VECTOR = List.of(
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity(),
        Translation2d.identity()
    );

    protected Drive() {
        super(NAME);
        mPeriodicIO = new PeriodicIO();

        openLoopRampRate = Constants.kOpenLoopRampRate;

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));

        mPigeon.configFactoryDefault();
    }

    @Override
    public double getHeadingDegrees() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    @Override
    public abstract double getDesiredHeading();

    @Override
    public double getKP() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        return (!factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .pidConfig.getOrDefault(pidSlot, defaultPIDConfig)
                .kP
            : 0.0;
    }

    @Override
    public double getKI() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kI = 0.0;
        return (!factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .pidConfig.getOrDefault(pidSlot, defaultPIDConfig)
                .kI
            : 0.0;
    }

    @Override
    public double getKD() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kD = 0.0;
        return (!factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .pidConfig.getOrDefault(pidSlot, defaultPIDConfig)
                .kD
            : 0.0;
    }

    @Override
    public double getKF() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kF = 0.0;
        return (!factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .pidConfig.getOrDefault(pidSlot, defaultPIDConfig)
                .kF
            : 0.0;
    }

    @Singleton
    public static class PeriodicIO {

        // INPUTS
        public double timestamp;
        public Rotation2d gyro_heading = Rotation2d.identity();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        public double drive_distance_inches;
        public double velocity_inches_per_second = 0;
        public double left_position_ticks;
        public double right_position_ticks;
        public double left_velocity_ticks_per_100ms;
        public double right_velocity_ticks_per_100ms;
        // no_offset = Relative to initial position, unaffected by reset
        double left_error;
        double right_error;

        // SWERVE
        public double forward;
        public double strafe;
        public double rotation;
        public boolean low_power;
        public boolean field_relative = factory.getConstant("teleopFieldCentric") > 0;
        public boolean use_heading_controller;

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public double[] wheel_speeds = new double[] { 0, 0, 0, 0 };
        public Rotation2d[] wheel_azimuths = new Rotation2d[] {
            Rotation2d.identity(),
            Rotation2d.identity(),
            Rotation2d.identity(),
            Rotation2d.identity(),
        };
        public Rotation2d desired_heading = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(
            Pose2dWithCurvature.identity()
        );
        public Translation2d drive_vector = Translation2d.identity();
    }

    @Override
    public void writePeriodicOutputs() {
        super.writePeriodicOutputs();
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                    synchronized (Drive.this) {
                        stop();
                        setBrakeMode(false);
                    }
                    lastUpdateTimestamp = timestamp;
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        mPeriodicIO.timestamp = timestamp;
                        switch (mDriveControlState) {
                            case OPEN_LOOP:
                                updateOpenLoopPeriodic(timestamp);
                                break;
                            case TRAJECTORY_FOLLOWING:
                                updatePathFollower(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " +
                                    mDriveControlState
                                );
                                break;
                        }
                    }
                    lastUpdateTimestamp = timestamp;
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    protected abstract void updateOpenLoopPeriodic(double timestamp);

    protected abstract void updatePathFollower(double timestamp);

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    protected static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    protected static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToRotations(inches_per_second) * DRIVE_ENCODER_PPR / 10.0;
    }

    protected static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     * @param signal
     */
    public abstract void setOpenLoop(DriveSignal signal);

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
    }

    public abstract void setTeleopInputs(
        double forward,
        double strafe,
        double rotation,
        boolean low_power,
        boolean use_heading_controller
    );

    public double getOpenLoopRampRate() {
        return this.openLoopRampRate;
    }

    /**
     * Configure talons for velocity control
     */
    public abstract void setVelocity(List<Translation2d> driveVectors);

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public void setBrakeMode(boolean on) {
        mIsBrakeMode = on;
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized Rotation2d getHeadingRelativeToInitial() {
        return mPeriodicIO.gyro_heading_no_offset;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset =
            heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.desired_heading = heading;
    }

    public synchronized void resetPigeon() {
        mPigeon.setYaw(0);
        mPigeon.setFusedHeading(0);
        mPigeon.setAccumZAngle(0);
    }

    public DriveControlState getDriveControlState() {
        return mDriveControlState;
    }

    public abstract void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory,
        Rotation2d targetHeading
    );

    public abstract boolean isDoneWithTrajectory();

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        TRAJECTORY_FOLLOWING,
    }

    @Override
    public void zeroSensors() {
        zeroSensors(Pose2d.identity());
    }

    public abstract void zeroSensors(Pose2d pose);

    public boolean hasPigeonResetOccurred() {
        return mPigeon.hasResetOccurred();
    }

    @Override
    public abstract void stop();

    @Override
    public abstract boolean checkSystem();

    @Override
    public double getFieldXDistance() {
        return mRobotState.getEstimatedX();
    }

    @Override
    public double getFieldYDistance() {
        return mRobotState.getEstimatedY();
    }

    @Override
    public double getFieldDesiredXDistance() {
        return 0;
    }

    @Override
    public double getFieldYDesiredYDistance() {
        return 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getDriveControlState().toString(),
            null
        );

        driveHelperChooser = new SendableChooser<>();
        driveHelperChooser.addOption("Cheesy Drive", new CheesyDriveHelper());
        driveHelperChooser.setDefaultOption("Swerve Classic", new SwerveDriveHelper());
        SmartDashboard.putData("Drive Algorithm", driveHelperChooser);

        SmartDashboard.putNumber("Drive/Vector Direction", 0);
        SmartDashboard.putNumber("Drive/Robot Velocity", 0);
        SmartDashboard.putNumber("Drive/OpenLoopRampRate", this.openLoopRampRate);
        SmartDashboard
            .getEntry("Drive/OpenLoopRampRate")
            .addListener(
                notification -> setOpenLoopRampRate(notification.value.getDouble()),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );

        SmartDashboard.putBoolean("Drive/Zero Sensors", false);
        SmartDashboard
            .getEntry("Drive/Zero Sensors")
            .addListener(
                entryNotification -> {
                    if (entryNotification.value.getBoolean()) {
                        zeroSensors();
                        entryNotification.getEntry().setBoolean(false);
                    }
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}
