package com.team1816.season.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.season.AutoModeSelector;
import com.team1816.season.Constants;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveSignal;
import com.team254.lib.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    private static final String NAME = "drivetrain";

    private static SwerveDrive INSTANCE;
    public SwerveModule[] swerveModules;


    @Inject
    private static SwerveHeadingController headingController;

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Odometry variables
    private Pose2d pose = new Pose2d();
    private Pose2d startingPosition = new Pose2d();
    private double lastUpdateTimestamp = 0;
    private SwerveDriveOdometry swerveOdometry;

    // Path control variables
    boolean hasStartedFollowing = false;
    boolean modulesReady = false;
    boolean alwaysConfigureModules = false;
    boolean moduleConfigRequested = false;
    private boolean wantReset = false;

    public SwerveDrive() {
        super();
        swerveModules = new SwerveModule[4];

        // start all Talons in open loop mode
        swerveModules[Constants.Swerve.kFrontLeft] =
            factory.getSwerveModule(
                NAME,
                "frontLeft",
                Constants.kFrontLeftModulePosition
            );
        swerveModules[Constants.Swerve.kFrontRight] =
            factory.getSwerveModule(
                NAME,
                "frontRight",
                Constants.kFrontRightModulePosition
            );
        swerveModules[Constants.Swerve.kBackLeft] =
            factory.getSwerveModule(NAME,
                "backLeft",
                Constants.kBackLeftModulePosition);
        swerveModules[Constants.Swerve.kBackRight] =
            factory.getSwerveModule(
                NAME,
                "backRight",
                Constants.kBackRightModulePosition
            );

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));
        mPigeon.configFactoryDefault();

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
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

    public void requireModuleConfiguration() {
        modulesReady = false;
    }

    public void alwaysConfigureModules() {
        alwaysConfigureModules = true;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            // calculate rotation based on left/right vel differences
            gyroDrift -=
                (
                    mPeriodicIO.left_velocity_ticks_per_100ms -
                    mPeriodicIO.right_velocity_ticks_per_100ms
                ) /
                robotWidthTicks;
            //mPeriodicIO.gyro_heading_no_offset = getDesiredRotation2d().rotateBy(Rotation2d.fromDegrees(gyroDrift));
        } else {
            mPeriodicIO.gyro_heading_no_offset =
                Rotation2d.fromDegrees(mPigeon.getFusedHeading());
        }
        mPeriodicIO.gyro_heading =
            mPeriodicIO.gyro_heading_no_offset.rotateBy(mGyroOffset);
        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
        for (SwerveModule module : swerveModules) {
            module.readPeriodicInputs();
        }
        swerveOdometry.update(mPeriodicIO.gyro_heading, getStates());

    }

    @Override
    public synchronized void writePeriodicOutputs() {

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
//                Constants.fieldRelative ?
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                    mPeriodicIO.forward,
                    mPeriodicIO.strafe,
                    mPeriodicIO.rotation,
                    getYaw()
                )
//                    : new ChassisSpeeds(
//                    mPeriodicIO.forward,
//                    mPeriodicIO.strafe,
//                    mPeriodicIO.rotation)
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Units.inches_to_meters(Constants.kPathFollowingMaxVel)); // TODO get swerve max speed in meters/s

        for(int i = 0; i < 4; i++) {
            // Gutted if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) b/c else statement after does same thing
            swerveModules[i].setDesiredState(swerveModuleStates[i], mDriveControlState == DriveControlState.OPEN_LOOP);
            //below used to be in its own for loop @ bottom - necessary?
            swerveModules[i].writePeriodicOutputs();
        }
    }

    public void setStartingPose(Pose2d pose) {
        this.startingPosition = pose;
    }

    @Override
    protected void updateOpenLoopPeriodic() {
        // no openLoop update needed
    }

    @Override
    protected void updateTrajectoryPeriodic(double timestamp) {
        // update desired pose from trajectory
        mPeriodicIO.desired_pose = mTrajectory.sample(timestamp).poseMeters;
    }

    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
//        SwerveDriveSignal swerveSignal = (SwerveDriveSignal) signal;
//
//        mPeriodicIO.wheel_speeds = swerveSignal.getWheelSpeeds();
//        mPeriodicIO.wheel_azimuths = swerveSignal.getWheelAzimuths();
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

        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    /**
     * Configure talons for velocity control
   */

//    public synchronized void setVelocity(List<Translation2d> driveVectors) {
//        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
//            setBrakeMode(false);
//            System.out.println("Switching to Velocity");
//        }
//        double[] speedsNorm = new double[4];
//        for (int i = 0; i < swerveModules.length; i++) {
//            mPeriodicIO.wheel_azimuths[i] = driveVectors.get(i).direction();
//            speedsNorm[i] = driveVectors.get(i).norm();
//            mPeriodicIO.wheel_speeds[i] =
//                inchesPerSecondToTicksPer100ms(
//                    driveVectors.get(i).norm() * Constants.kPathFollowingMaxVel
//                );
//        }
//        if (RobotBase.isSimulation()) {
//            mPeriodicIO.gyro_heading_no_offset.rotateBy(
//                Rotation2d.fromDegrees(
//                    SwerveKinematics.forwardKinematics(
//                        speedsNorm,
//                        mPeriodicIO.wheel_azimuths
//                    )
//                        .dtheta
//                )
//            );
//        }
//    }

    @Override
    public synchronized void setBrakeMode(boolean on) {
        super.setBrakeMode(on);
        for (SwerveModule module : swerveModules) {
            module.setDriveBrakeMode(on);
        }
    }
//
//    public void setWantReset(boolean wantReset) {
//        this.wantReset = wantReset;
//    }
//
//    public boolean wantsReset() {
//        return wantReset;
//    }

//    @Override
//    public synchronized void setTrajectory(
//        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory,
//        Rotation2d targetHeading
//    ) {
//        if (motionPlanner != null) {
//            hasStartedFollowing = false;
//            moduleConfigRequested = false;
//            System.out.println("Now setting trajectory");
//            setBrakeMode(true);
//            mOverrideTrajectory = false;
//            headingController.setSnapTarget(targetHeading.getDegrees());
//            motionPlanner.reset();
//            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
//            motionPlanner.setTrajectory(trajectory);
//        }
//    }

//    @Override
//    public void updatePathFollower(double timestamp) {
//        double rotationCorrection = headingController.updateRotationCorrection(
//            getHeadingDegrees(),
//            timestamp
//        );
//        updatePose(timestamp);
//        // alternatePoseUpdate(timestamp);
//
//        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
//            if (!motionPlanner.isDone()) {
//                Translation2d driveVector = motionPlanner.update(timestamp, pose);
//
//                if (!hasStartedFollowing && wantReset) {
//                    zeroSensors(startingPosition);
//                    System.out.println("Position reset for auto");
//                    hasStartedFollowing = true;
//                    wantReset = false;
//                }
//
//                //                                System.out.println("DRIVE VECTOR" + driveVector);
//
//                mPeriodicIO.forward = driveVector.x();
//                mPeriodicIO.strafe = driveVector.y();
//                mPeriodicIO.rotation = 0;
//
//                double rotationInput = Util.deadBand(
//                    Util.limit(
//                        rotationCorrection * rotationScalar * driveVector.norm(),
//                        motionPlanner.getMaxRotationSpeed()
//                    ),
//                    0.01
//                );
//
//                mPeriodicIO.error = motionPlanner.error();
//                mPeriodicIO.path_setpoint = motionPlanner.setpoint();
//                mPeriodicIO.drive_vector = driveVector;
//                if (!mOverrideTrajectory) {
//                    //                    System.out.println("ROTATIONINPUT==" + rotationInput);
//                    setVelocity(
//                        SwerveKinematics.updateDriveVectors(
//                            driveVector,
//                            rotationInput,
//                            pose,
//                            robotCentric
//                        )
//                    );
//                }
//            } else {
//                setVelocity(ZERO_DRIVE_VECTOR);
//                if (alwaysConfigureModules) requireModuleConfiguration();
//            }
//        } else {
//            DriverStation.reportError("drive is not in path following state", false);
//        }
//    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(pose.getRotation());
        resetOdometry(pose);

        for (SwerveModule module : swerveModules) {
            if (module != null) {
                module.zeroSensors();
            }
        }
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



    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.inches_to_meters(Constants.kPathFollowingMaxVel)); // TODO max speeeeed like above

        for(int i = 0; i < 4; i++){
            swerveModules[i].setDesiredState(desiredStates[i], false);
        }
    }

    @Override
    public void updateTrajectoryVelocities(Double aDouble, Double aDouble1) {

    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    @Override
    public void startTrajectory(Trajectory trajectory) {
        mTrajectory = trajectory;
        swerveOdometry.resetPosition(
            trajectory.getInitialPose(),
            trajectory.getInitialPose().getRotation()
        );
        updateRobotPose();
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        setBrakeMode(true);
        mOverrideTrajectory = false;
    }

    private void updateRobotPose() {
        mRobotState.field.setRobotPose(swerveOdometry.getPoseMeters());
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        mPigeon.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(ypr[0]); // TODO keep in possibility of pigeon being reversed? (Constants.Swerve.invertGyro)
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(SwerveDriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean modulesPassed = true;
        for (SwerveModule module : swerveModules) {
            modulesPassed = modulesPassed && module.checkSystem();
        }

        boolean checkPigeon = mPigeon == null;

        System.out.println(modulesPassed && checkPigeon);
        return modulesPassed;
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

