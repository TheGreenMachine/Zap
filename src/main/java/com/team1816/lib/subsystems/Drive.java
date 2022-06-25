package com.team1816.lib.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.LedManager;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;

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

    // states
    protected ControlState controlState = ControlState.OPEN_LOOP;
    protected boolean isBraking;
    protected boolean isSlowMode;
    protected PeriodicIO mPeriodicIO;

    // Trajectory
    protected double trajectoryStartTime = 0;
    protected Trajectory trajectory;

    // Simulator
    protected double gyroDrift;
    protected final double tickRatioPerLoop = Constants.kLooperDt / .01d;

    // Constants
    protected final double heatThreshold = factory.getConstant(
        NAME,
        "heatThreshold",
        100
    );
    public static final double maxTicks = factory.getConstant(
        NAME,
        "maxTicks"
    );
    public static final double driveEncPPR = factory.getConstant(NAME, "encPPR");

    protected Drive() {
        super(NAME);
        mPeriodicIO = new PeriodicIO();
    }

    @Singleton
    public static class PeriodicIO {

        // INPUTS
        public double timestamp;
        public Rotation2d gyro_heading = Constants.EmptyRotation;
        // no_offset = Relative to initial position, unaffected by reset
        public Rotation2d actualHeading = Constants.EmptyRotation;
        public Rotation2d desired_heading = new Rotation2d();
        public Pose2d desired_pose = new Pose2d();
        public ChassisSpeeds chassisSpeed = new ChassisSpeeds();
    }

    // calls periodic methods in swerve/tank based on current control state
    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        mPeriodicIO.timestamp = timestamp;
                        switch (controlState) {
                            case OPEN_LOOP:
                                updateOpenLoopPeriodic();
                                break;
                            case TRAJECTORY_FOLLOWING:
                                updateTrajectoryPeriodic(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " +
                                        controlState
                                );
                                break;
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    // autonomous (trajectory following)
    public void startTrajectory(
        Trajectory trajectory,
        List<Rotation2d> headings
    ){
        controlState = ControlState.TRAJECTORY_FOLLOWING;
        trajectoryStartTime = 0;
        this.trajectory = trajectory;
        updateRobotState();
    };

    public Pose2d getPose() {
        return robotState.fieldToVehicle;
    }

    public void updateTrajectoryPeriodic(double timestamp) {
        if (trajectoryStartTime == 0) trajectoryStartTime = timestamp;
        // update desired pose from trajectory
        mPeriodicIO.desired_pose =
            trajectory.sample(timestamp - trajectoryStartTime).poseMeters;
        mPeriodicIO.desired_heading = mPeriodicIO.desired_pose.getRotation();
    }

    protected void updateOpenLoopPeriodic() {}

    protected abstract void updateRobotState();

    /**
     * Configure talons for open loop control
     * @param signal
     */
    public abstract void setOpenLoop(DriveSignal signal);

    // general setters
    public abstract void setTeleopInputs(
        double forward,
        double strafe,
        double rotation
    );

    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    // getters
    @Override
    public abstract PIDSlotConfiguration getPIDConfig();

    @Override
    public double getDesiredHeading() {
        return mPeriodicIO.desired_heading.getDegrees();
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.actualHeading;
    }

    @Override
    public double getHeadingDegrees() {
        return mPeriodicIO.actualHeading.getDegrees();
    }

    public ControlState getControlState() {
        return controlState;
    }

    @Override
    public double getFieldXDistance() {
        return Units.metersToInches(getPose().getX() - Constants.StartingPose.getX());
    }

    @Override
    public double getFieldYDistance() {
        return Units.metersToInches(getPose().getY() - Constants.StartingPose.getY());
    }

    @Override
    public double getFieldDesiredXDistance() {
        if (mPeriodicIO.desired_pose.getX() == 0) return 0;
        return Units.metersToInches(
            mPeriodicIO.desired_pose.getX() - Constants.StartingPose.getX()
        );
    }

    @Override
    public double getFieldDesiredYDistance() {
        if (mPeriodicIO.desired_pose.getY() == 0) return 0;
        return Units.metersToInches(
            mPeriodicIO.desired_pose.getY() - Constants.StartingPose.getY()
        );
    }

    // calls used during initialization || game phase change
    public boolean isBraking() {
        return isBraking;
    }

    public abstract void setBraking(boolean on);

    public abstract void resetOdometry(Pose2d pose);

    @Override
    public void zeroSensors() {
        zeroSensors(getPose());
    }

    public abstract void zeroSensors(Pose2d pose);

    @Override
    public abstract void stop();

    // other
    @Override
    public abstract boolean checkSystem();

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getControlState().toString(),
            null
        );
    }

    public synchronized double getTrajectoryTimestamp() {
        return mPeriodicIO.timestamp - trajectoryStartTime;
    }

    public void simulateGyroOffset(){
        // simulates rotation by computing the rotational motion per interval
        double simGyroOffset =
            mPeriodicIO.chassisSpeed.omegaRadiansPerSecond * tickRatioPerLoop;
        // increment gyro drift
        gyroDrift -= 0;
        Infrastructure.simulateGyro(simGyroOffset, gyroDrift);
    }

    public enum ControlState {
        OPEN_LOOP, // open loop voltage control
        TRAJECTORY_FOLLOWING,
    }
}
