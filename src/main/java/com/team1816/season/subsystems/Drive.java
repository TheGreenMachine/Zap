package com.team1816.season.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.subsystems.TrackableDrivetrain;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public abstract class Drive
    extends Subsystem
    implements TrackableDrivetrain, PidProvider {

    public abstract void updateTrajectoryVelocities(Double aDouble, Double aDouble1);

    public abstract edu.wpi.first.math.geometry.Pose2d getPose();

    public abstract void startTrajectory(Trajectory initialPose);

    public interface Factory {
        Drive getInstance();
    }

    public static final String NAME = "drivetrain";

    // Components
    @Inject
    protected static LedManager ledManager;

    protected PigeonIMU mPigeon;

    // control states
    protected DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;

    @Inject
    protected static RobotState mRobotState;

    // Odometry variables
    protected double lastUpdateTimestamp = 0;

    // hardware states
    protected String pidSlot = "slot0";
    protected boolean mIsBrakeMode;
    protected Rotation2d mGyroOffset = new Rotation2d();
    protected double openLoopRampRate;

    protected PeriodicIO mPeriodicIO;
    protected boolean mOverrideTrajectory = false;

    protected boolean isSlowMode;

    // Simulator
    protected double gyroDrift;
    protected final double robotWidthTicks =
        inchesPerSecondToTicksPer100ms(Constants.kDriveWheelTrackWidthInches) * Math.PI;

    // Constants
    public static final double maxVelTicksPer100ms = factory.getConstant("maxTicks");
    public static final double DRIVE_ENCODER_PPR = factory.getConstant(NAME, "encPPR");
//    public static final List<Translation2d> ZERO_DRIVE_VECTOR = List.of(
//        new Translation2d(),
//        new Translation2d(),
//        new Translation2d(),
//        new Translation2d()
//    );

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
        public Rotation2d gyro_heading = new Rotation2d();
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d gyro_heading_no_offset = new Rotation2d();
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
        public double rotation;

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;

        public Rotation2d desired_heading = new Rotation2d();
        public Pose2d desired_pose = new Pose2d();
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
                                updateOpenLoopPeriodic();
                                break;
                            case TRAJECTORY_FOLLOWING:
                                updateTrajectoryPeriodic(timestamp);
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

    protected abstract void updateOpenLoopPeriodic();

    protected abstract void updateTrajectoryPeriodic(double timestamp);

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    protected static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    protected static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double metersPerSecondToTicksPer100ms(double meters_per_second) {
        return inchesPerSecondToTicksPer100ms(Units.meters_to_inches(meters_per_second));
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
            heading.rotateBy(
                Rotation2d.fromDegrees(mPigeon.getFusedHeading()).unaryMinus()
            );
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

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        TRAJECTORY_FOLLOWING,
    }

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
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getDriveControlState().toString(),
            null
        );
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
