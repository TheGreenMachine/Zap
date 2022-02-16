package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team1816.season.AutoModeSelector;
import com.team1816.season.Constants;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    public static final String NAME = "drivetrain";

    public SwerveModule[] swerveModules;

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Odometry variables
    private SwerveDriveOdometry swerveOdometry;

    public SwerveDrive() {
        super();
        swerveModules = new SwerveModule[4];

        // start all Talons in open loop mode
        swerveModules[Constants.Swerve.kFrontLeft] =
            factory.getSwerveModule(NAME, "frontLeft");
        swerveModules[Constants.Swerve.kFrontRight] =
            factory.getSwerveModule(NAME, "frontRight");
        swerveModules[Constants.Swerve.kBackLeft] =
            factory.getSwerveModule(NAME, "backLeft");
        swerveModules[Constants.Swerve.kBackRight] =
            factory.getSwerveModule(NAME, "backRight");

        //        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));
        //        mPigeon.configFactoryDefault();

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        swerveOdometry =
            new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getHeading());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(mDriveControlState == DriveControlState.OPEN_LOOP){ // autonomous (Trajectory_Following) loop is in setModuleStates
            SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.desiredModuleStates,
                Units.inchesToMeters(Constants.kPathFollowingMaxVel)
            ); // TODO get swerve max speed in meters/s
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(mPeriodicIO.desiredModuleStates[i], true);
            }
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            // calculate rotation with gyro drift
            gyroDrift -= 0;
            mPeriodicIO.gyro_heading_no_offset =
                getDesiredRotation2d().rotateBy(Rotation2d.fromDegrees(gyroDrift));
        } else {
            mPeriodicIO.gyro_heading_no_offset =
                Rotation2d.fromDegrees(mPigeon.getFusedHeading());
        }
        mPeriodicIO.gyro_heading =
            mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
        SwerveModuleState[] states = new SwerveModuleState[4]; // why do we create 4 new states in ever single loop through the readPeriodic?
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }

        mPeriodicIO.actualModuleStates = states;
        swerveOdometry.update(mPeriodicIO.gyro_heading, states);
        updateRobotPose();
    }

    // autonomous (trajectory following)
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        mPeriodicIO.timestamp = 0;
        mTrajectoryStart = 0;
        mTrajectory = trajectory;
        mHeadings = headings;
        mTrajectoryIndex = 0;
        swerveOdometry.resetPosition(
            trajectory.getInitialPose(),
            trajectory.getInitialPose().getRotation()
        );
        updateRobotPose();
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        setBrakeMode(true);
        mOverrideTrajectory = false;
    }

    @Override // not used in swerve
    public void updateTrajectoryVelocities(Double aDouble, Double aDouble1) {}

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if(mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING){
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            Units.inchesToMeters(Constants.kPathFollowingMaxVel)
        );
        mPeriodicIO.desiredModuleStates = desiredStates;
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], false);
        }
    }

    @Override
    public Rotation2d getTrajectoryHeadings() {
        if (mHeadings == null) {
            System.out.println("headings are empty!");
            return Constants.EmptyRotation;
        } else if (mTrajectoryIndex > mHeadings.size() - 1) {
            System.out.println("heck the headings aren't long enough");
            return Constants.EmptyRotation;
        }
        if (
            getTrajectoryTimestamp() >
            mTrajectory.getStates().get(mTrajectoryIndex).timeSeconds ||
            mTrajectoryIndex == 0
        ) mTrajectoryIndex++;
        if (mTrajectoryIndex >= mHeadings.size()) {
            System.out.println(mHeadings.get(mHeadings.size() - 1) + " = max");
            return mHeadings.get(mHeadings.size() - 1);
        }
        double timeBetweenPoints =
            (
                mTrajectory.getStates().get(mTrajectoryIndex).timeSeconds -
                mTrajectory.getStates().get(mTrajectoryIndex - 1).timeSeconds
            );
        Rotation2d heading;
        heading =
            mHeadings
                .get(mTrajectoryIndex - 1)
                .interpolate(
                    mHeadings.get(mTrajectoryIndex),
                    getTrajectoryTimestamp() / timeBetweenPoints
                );
        //        System.out.println(heading.getDegrees() + "aaaaa");
        //        mPeriodicIO.totalRotation = heading.getRadians();
        return heading;
    }

    public Pose2d getPose() {
        return mRobotState.field_to_vehicle; // swerveOdometry.getPoseMeters();
    }

    private void updateRobotPose() {
        mRobotState.field_to_vehicle = swerveOdometry.getPoseMeters();
    }

    @Override
    protected void updateOpenLoopPeriodic() {
        // no openLoop update needed
    }

    // general setters
    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(new SwerveModuleState(), true);
                mPeriodicIO.desiredModuleStates[i] = new SwerveModuleState();
                mPeriodicIO.actualModuleStates[i] = new SwerveModuleState();
            }
        }
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
        mPeriodicIO.use_heading_controller = use_heading_controller;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forward *
                Units.inchesToMeters(Constants.kPathFollowingMaxVel), // test this out  -
            strafe * Units.inchesToMeters(Constants.kPathFollowingMaxVel),
            rotation * (Constants.kMaxAngularSpeed),
            mPeriodicIO.gyro_heading // ignore gyro
        );
        System.out.println("Set TeleopInputs " + speeds);
        mPeriodicIO.desiredModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            speeds
        );
    }

    @Override
    public synchronized void setBrakeMode(boolean on) {
        super.setBrakeMode(on);
        for (SwerveModule module : swerveModules) {
            module.setDriveBrakeMode(on);
        }
    }

    // general getters
    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public SwerveModuleState[] getStates() {
        return mPeriodicIO.actualModuleStates;
    }

    @Override
    public Rotation2d getDesiredRotation2d() {
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return getTrajectoryHeadings();
        }
        return mPeriodicIO.desired_heading;
    }

    @Override
    public double getLeftVelocityNativeUnits() {
        return 0;
    }

    @Override
    public double getRightVelocityNativeUnits() {
        return 0;
    }

    @Override
    public double getLeftVelocityDemand() {
        return 0;
    }

    @Override
    public double getRightVelocityDemand() {
        return 0;
    }

    @Override
    public double getLeftVelocityError() {
        return 0;
    }

    @Override
    public double getRightVelocityError() {
        return 0;
    }

    @Override
    public double getDesiredHeading() {
        return getDesiredRotation2d().getDegrees();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getHeading());
    }

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(pose.getRotation());
        resetOdometry(pose);
        mRobotState.field.setRobotPose(Constants.StartingPose);
        //        if (mPigeon.getLastError() != ErrorCode.OK) {
        //            // BadLog.createValue("PigeonErrorDetected", "true");
        //            System.out.println(
        //                "Error detected with Pigeon IMU - check if the sensor is present and plugged in!"
        //            );
        //            System.out.println("Defaulting to drive straight mode");
        //            AutoModeSelector.getInstance().setHardwareFailure(true);
        //        } else {
        autoModeSelector.setHardwareFailure(false);
        //        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(SwerveDriveSignal.NEUTRAL);
        mTrajectoryIndex = 0;
    }

    // other
    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean modulesPassed = true;

        boolean checkPigeon = mPigeon == null;

        return true; // not actually doing anything lol
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //        SmartDashboard.putBoolean(
        //            "Drive/TeleopFieldCentric",
        //            this.mPeriodicIO.field_relative
        //        );
        //        SmartDashboard
        //            .getEntry("Drive/TeleopFieldCentric")
        //            .addListener(
        //                notification -> {
        //                    this.mPeriodicIO.field_relative = notification.value.getBoolean();
        //                },
        //                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        //            );
    }
}
