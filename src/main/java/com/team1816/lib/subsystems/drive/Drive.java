package com.team1816.lib.subsystems.drive;

import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    protected static LedManager ledManager;

    // states
    protected ControlState controlState = ControlState.OPEN_LOOP;
    protected Rotation2d actualHeading = Constants.EmptyRotation;
    protected Rotation2d desiredHeading = new Rotation2d(); // only updated in trajectory following
    protected Pose2d desiredPose = new Pose2d(); // only updated in trajectory following
    protected ChassisSpeeds chassisSpeed = new ChassisSpeeds();

    protected boolean isBraking;
    protected boolean isSlowMode;

    // Trajectory
    protected double trajectoryStartTime = 0;

    protected Pose2d startingPose = Constants.kDefaultZeroingPose;
    protected Trajectory trajectory;
    protected static double timestamp;

    // Simulator
    protected double gyroDrift;
    protected final double tickRatioPerLoop = Constants.kLooperDt / .01d;

    // Constants
    protected final double heatThreshold = factory.getConstant(
        NAME,
        "heatThreshold",
        100
    );
    public static final double maxVelTicks100ms = factory.getConstant(
        NAME,
        "maxVelTicks100ms"
    );
    public static final double driveEncPPR = factory.getConstant(NAME, "encPPR");

    // Drivetrain characterization
    public static final double kDriveWheelTrackWidthInches = factory.getConstant(
        NAME,
        "trackWidth",
        22
    );
    public static final double kDriveWheelbaseLengthInches = factory.getConstant(
        NAME,
        "wheelbaseLength",
        22
    );
    public static final double kDriveWheelDiameterInches = factory.getConstant(
        NAME,
        "wheelDiameter"
    );
    public static final double kWheelCircumferenceInches =
        kDriveWheelDiameterInches * Math.PI;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;

    public static final double kDriveWheelTrackWidthMeters = Units.inchesToMeters(
        kDriveWheelTrackWidthInches
    );
    public static final double kDriveWheelbaseLengthMeters = Units.inchesToMeters(
        kDriveWheelbaseLengthInches
    );
    public static final double kDriveWheelDiameterMeters = Units.inchesToMeters(
        kDriveWheelDiameterInches
    );
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(
        kWheelCircumferenceInches
    );
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(
        kDriveWheelRadiusInches
    );
    public static double kTrackScrubFactor = factory.getConstant("kTrackScrubFactor");
    // Drive speed
    public static final double kPathFollowingMaxAccelMeters = factory.getConstant(
        NAME,
        "maxAccel",
        4
    );
    public static final double kPathFollowingMaxVelMeters = factory.getConstant(
        NAME,
        "maxVelPathFollowing"
    );
    public static final double kOpenLoopMaxVelMeters = factory.getConstant(
        NAME,
        "maxVelOpenLoop"
    );

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 4;
    public static final double kMaxAngularSpeed = factory.getConstant("maxRotVel"); // rad/sec
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
        2 * Math.PI;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared
    );

    @Inject
    public Drive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        ledManager = lm;
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
                        Drive.timestamp = timestamp;
                        switch (controlState) {
                            case OPEN_LOOP:
                                updateOpenLoopPeriodic();
                                break;
                            case TRAJECTORY_FOLLOWING:
                                updateTrajectoryPeriodic(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " + controlState
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
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        controlState = ControlState.TRAJECTORY_FOLLOWING;
        trajectoryStartTime = 0;
        this.trajectory = trajectory;
        updateRobotState();
    }

    public Pose2d getPose() {
        return robotState.fieldToVehicle;
    }

    public void updateTrajectoryPeriodic(double timestamp) {
        if (trajectory == null) {
            return;
        }
        if (trajectoryStartTime == 0) trajectoryStartTime = timestamp;
        // update desired pose from trajectory
        desiredPose = trajectory.sample(timestamp - trajectoryStartTime).poseMeters;
        desiredHeading = desiredPose.getRotation();
    }

    protected void updateOpenLoopPeriodic() {}

    protected abstract void updateRobotState();

    /**
     * Configure talons for open loop control
     * @param signal
     */
    public abstract void setOpenLoop(DriveSignal signal);

    // general setters
    public abstract void setTeleopInputs(double forward, double strafe, double rotation);

    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    public synchronized Rotation2d getActualHeading() {
        return actualHeading;
    }

    public ControlState getControlState() {
        return controlState;
    }

    @Override
    public abstract PIDSlotConfiguration getPIDConfig();

    @Override
    public double getDesiredHeadingDegrees() {
        return desiredHeading.getDegrees();
    }

    @Override
    public double getActualHeadingDegrees() {
        return actualHeading.getDegrees();
    }

    @Override
    public double getFieldXDistance() {
        return getPose().getX() - startingPose.getX();
    }

    @Override
    public double getFieldYDistance() {
        return getPose().getY() - startingPose.getY();
    }

    @Override
    public double getFieldDesiredXDistance() {
        return desiredPose.getX() - startingPose.getX();
    }

    @Override
    public double getFieldDesiredYDistance() {
        return desiredPose.getY() - startingPose.getY();
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

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public synchronized double getTrajectoryTimestamp() {
        return timestamp - trajectoryStartTime;
    }

    public void simulateGyroOffset() {
        // simulates rotation by computing the rotational motion per interval
        double simGyroOffset = chassisSpeed.omegaRadiansPerSecond * tickRatioPerLoop;
        // increment gyro drift
        gyroDrift -= 0;
        infrastructure.simulateGyro(simGyroOffset, gyroDrift);
    }

    public enum ControlState {
        OPEN_LOOP, // open loop voltage control
        TRAJECTORY_FOLLOWING,
    }
}
