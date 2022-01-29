package com.team1816.frcSeason.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.frcSeason.AutoModeSelector;
import com.team1816.frcSeason.Constants;
import com.team1816.frcSeason.SwerveKinematics;
import com.team1816.frcSeason.planners.SwerveMotionPlanner;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.SwerveDrivetrain;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveSignal;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    private static final String NAME = "drivetrain";

    private static SwerveDrive INSTANCE;

    // Controllers
    private final SwerveMotionPlanner motionPlanner;

    @Inject
    private static SwerveHeadingController headingController;

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Odometry variables
    private Pose2d pose = Pose2d.identity();
    private Pose2d startingPosition = Pose2d.identity();
    private double lastUpdateTimestamp = 0;

    // Path control variables
    boolean hasStartedFollowing = false;
    boolean modulesReady = false;
    boolean alwaysConfigureModules = false;
    boolean moduleConfigRequested = false;
    private boolean wantReset = false;

    public SwerveDrive() {
        super();
        swerveModules = new SwerveModule[4];

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // start all Talons in open loop mode
        swerveModules[SwerveModule.kFrontLeft] =
            factory.getSwerveModule(
                NAME,
                "frontLeft",
                Constants.kFrontLeftModulePosition
            );
        swerveModules[SwerveModule.kFrontRight] =
            factory.getSwerveModule(
                NAME,
                "frontRight",
                Constants.kFrontRightModulePosition
            );
        swerveModules[SwerveModule.kBackLeft] =
            factory.getSwerveModule(NAME, "backLeft", Constants.kBackLeftModulePosition);
        swerveModules[SwerveModule.kBackRight] =
            factory.getSwerveModule(
                NAME,
                "backRight",
                Constants.kBackRightModulePosition
            );

        setOpenLoopRampRate(Constants.kOpenLoopRampRate);

        mPigeon = new PigeonIMU((int) factory.getConstant(NAME, "pigeonId", -1));

        mPigeon.configFactoryDefault();

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        motionPlanner = new SwerveMotionPlanner();
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
        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            return headingController.getTargetHeading();
        }
        return mPeriodicIO.desired_heading.getDegrees();
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
            var rot2d = new edu.wpi.first.math.geometry.Rotation2d(
                mPeriodicIO.rotation //still jank
            );
            var xPos =
                Units.inches_to_meters(mRobotState.getEstimatedX()) +
                Constants.StartingPose.getTranslation().x();
            var yPos =
                Units.inches_to_meters(mRobotState.getEstimatedY()) +
                Constants.StartingPose.getTranslation().y();
            mRobotState.field.setRobotPose(xPos, yPos, rot2d);
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
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < swerveModules.length; i++) {
            if (swerveModules[i] != null) {
                if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    // TODO: 5/5/21 fix
                    if (
                        Util.shouldReverse(
                            swerveModules[i].getAngle().getDegrees(),
                            mPeriodicIO.wheel_azimuths[i].getDegrees()
                        )
                    ) {
                        swerveModules[i].setOpenLoop(
                                -mPeriodicIO.wheel_speeds[i],
                                mPeriodicIO.wheel_azimuths[i].rotateBy(
                                        Rotation2d.fromDegrees(180)
                                    )
                            );
                    } else {
                        swerveModules[i].setOpenLoop(
                                mPeriodicIO.wheel_speeds[i],
                                mPeriodicIO.wheel_azimuths[i]
                            );
                    }
                } else if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
                    if (
                        Util.shouldReverse(
                            swerveModules[i].getAngle().getDegrees(),
                            mPeriodicIO.wheel_azimuths[i].getDegrees()
                        )
                    ) {
                        swerveModules[i].setVelocity(
                                -mPeriodicIO.wheel_speeds[i],
                                mPeriodicIO.wheel_azimuths[i].rotateBy(
                                        Rotation2d.fromDegrees(180)
                                    )
                            );
                    } else {
                        swerveModules[i].setVelocity(
                                mPeriodicIO.wheel_speeds[i],
                                mPeriodicIO.wheel_azimuths[i]
                            );
                    }
                }
                swerveModules[i].writePeriodicOutputs();
            }
        }
    }

    /** The tried and true algorithm for keeping track of position */
    public synchronized void updatePose(double timestamp) {
        double x = 0.0;
        double y = 0.0;
        Rotation2d heading = getHeading();

        double averageDistance = 0.0;
        double[] distances = new double[4];
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule m = swerveModules[i];
            m.updatePose(heading);
            double distance = m
                .getEstimatedRobotPose()
                .getTranslation()
                .translateBy(pose.getTranslation().inverse())
                .norm();
            distances[i] = distance;
            averageDistance += distance;
        }
        averageDistance /= swerveModules.length;

        int minDevianceIndex = 0;
        double minDeviance = 100.0;
        List<SwerveModule> modulesToUse = new ArrayList<>();
        for (int i = 0; i < swerveModules.length; i++) {
            SwerveModule m = swerveModules[i];
            double deviance = Math.abs(distances[i] - averageDistance);
            if (deviance < minDeviance) {
                minDeviance = deviance;
                minDevianceIndex = i;
            }
            if (deviance <= 0.01) {
                modulesToUse.add(m);
            }
        }

        if (modulesToUse.isEmpty()) {
            modulesToUse.add(swerveModules[minDevianceIndex]);
        }

        //SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        Pose2d updatedPose = new Pose2d(
            new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()),
            heading
        );
        double deltaPos = updatedPose
            .getTranslation()
            .translateBy(pose.getTranslation().inverse())
            .norm();
        mPeriodicIO.drive_distance_inches += deltaPos;
        mPeriodicIO.velocity_inches_per_second =
            deltaPos / (timestamp - lastUpdateTimestamp);
        pose = updatedPose;
        System.out.println(updatedPose);
        for (SwerveModule module : swerveModules) {
            module.resetPose(pose);
        }
    }

    public synchronized void alternatePoseUpdate(double timestamp) {
        double x = 0.0;
        double y = 0.0;
        Rotation2d heading = Rotation2d.fromDegrees(getHeadingDegrees()); // temporary heading, some yaw calculation is being done here

        double[] distances = new double[4];

        for (int i = 0; i < 4; i++) {
            swerveModules[i].updatePose(heading);
            double distance =
                swerveModules[i].getEstimatedRobotPose()
                    .getTranslation()
                    .distance(pose.getTranslation());
            distances[i] = distance;
        }

        Arrays.sort(distances); // Doing some kind of sort for some reason, not sure why

        List<SwerveModule> modulesToUse = new ArrayList<>();
        double firstDifference = distances[1] - distances[0];
        double secondDifference = distances[2] - distances[1];
        double thirdDifference = distances[3] - distances[2];

        if (secondDifference > (1.5 * firstDifference)) {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
        } else if (thirdDifference > (1.5 * firstDifference)) {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
            modulesToUse.add(swerveModules[2]);
        } else {
            modulesToUse.add(swerveModules[0]);
            modulesToUse.add(swerveModules[1]);
            modulesToUse.add(swerveModules[2]);
            modulesToUse.add(swerveModules[3]);
        }

        SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        Pose2d updatedPose = new Pose2d(
            new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()),
            heading
        );
        double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
        mPeriodicIO.drive_distance_inches += deltaPos;
        mPeriodicIO.velocity_inches_per_second =
            deltaPos / (timestamp - lastUpdateTimestamp);
        pose = updatedPose;

        for (SwerveModule mModule : swerveModules) {
            mModule.resetPose(pose);
        }
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        mPeriodicIO.drive_distance_inches = 0;
        mPeriodicIO.velocity_inches_per_second = 0;
        for (SwerveModule module : swerveModules) {
            module.resetPose(pose);
        }
    }

    public void setStartingPose(Pose2d pose) {
        this.startingPosition = pose;
    }

    @Override
    protected void updateOpenLoopPeriodic(double timestamp) {
        var driveHelper = driveHelperChooser.getSelected();
        setOpenLoop(
            driveHelper.calculateDriveSignal(
                mPeriodicIO.forward,
                mPeriodicIO.strafe,
                mPeriodicIO.rotation,
                mPeriodicIO.low_power,
                mPeriodicIO.field_relative,
                mPeriodicIO.use_heading_controller
            )
        );
        updatePose(timestamp);
    }

    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        SwerveDriveSignal swerveSignal = (SwerveDriveSignal) signal;

        mPeriodicIO.wheel_speeds = swerveSignal.getWheelSpeeds();
        mPeriodicIO.wheel_azimuths = swerveSignal.getWheelAzimuths();
    }

    @Override
    public void setOpenLoopRampRate(double openLoopRampRate) {
        super.setOpenLoopRampRate(openLoopRampRate);
        for (SwerveModule module : swerveModules) {
            module.setOpenLoopRampRate(openLoopRampRate);
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

        mPeriodicIO.forward = forward;
        mPeriodicIO.strafe = strafe;
        mPeriodicIO.rotation = rotation;
        mPeriodicIO.low_power = low_power;
        mPeriodicIO.use_heading_controller = use_heading_controller;
    }

    /**
     * Configure talons for velocity control
     */
    @Override
    public synchronized void setVelocity(List<Translation2d> driveVectors) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to Velocity");
        }
        double[] speedsNorm = new double[4];
        for (int i = 0; i < swerveModules.length; i++) {
            mPeriodicIO.wheel_azimuths[i] = driveVectors.get(i).direction();
            speedsNorm[i] = driveVectors.get(i).norm();
            mPeriodicIO.wheel_speeds[i] =
                inchesPerSecondToTicksPer100ms(
                    driveVectors.get(i).norm() * Constants.kPathFollowingMaxVel
                );
        }
        if (RobotBase.isSimulation()) {
            mPeriodicIO.gyro_heading_no_offset.rotateBy(
                Rotation2d.fromDegrees(
                    SwerveKinematics.forwardKinematics(
                        speedsNorm,
                        mPeriodicIO.wheel_azimuths
                    )
                        .dtheta
                )
            );
        }
    }

    @Override
    public synchronized void setBrakeMode(boolean on) {
        super.setBrakeMode(on);
        for (SwerveModule module : swerveModules) {
            module.setDriveBrakeMode(on);
        }
    }

    public synchronized double[] getModuleVelocities() {
        double[] ret_val = new double[swerveModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = swerveModules[i].getLinearVelocity();
        }

        return ret_val;
    }

    public synchronized Rotation2d[] getModuleAzimuths() {
        Rotation2d[] ret_val = new Rotation2d[swerveModules.length];
        for (int i = 0; i < ret_val.length; i++) {
            ret_val[i] = swerveModules[i].getAngle();
        }

        return ret_val;
    }

    public void setWantReset(boolean wantReset) {
        this.wantReset = wantReset;
    }

    public boolean wantsReset() {
        return wantReset;
    }

    @Override
    public synchronized void setTrajectory(
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory,
        Rotation2d targetHeading
    ) {
        if (motionPlanner != null) {
            hasStartedFollowing = false;
            moduleConfigRequested = false;
            System.out.println("Now setting trajectory");
            setBrakeMode(true);
            mOverrideTrajectory = false;
            headingController.setSnapTarget(targetHeading.getDegrees());
            motionPlanner.reset();
            mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
            motionPlanner.setTrajectory(trajectory);
        }
    }

    @Override
    public boolean isDoneWithTrajectory() {
        if (mDriveControlState != DriveControlState.TRAJECTORY_FOLLOWING) {
            return false;
        }
        return motionPlanner.isDone() || mOverrideTrajectory;
    }

    @Override
    public void updatePathFollower(double timestamp) {
        double rotationCorrection = headingController.updateRotationCorrection(
            getHeadingDegrees(),
            timestamp
        );
        updatePose(timestamp);
        // alternatePoseUpdate(timestamp);

        if (mDriveControlState == DriveControlState.TRAJECTORY_FOLLOWING) {
            if (!motionPlanner.isDone()) {
                Translation2d driveVector = motionPlanner.update(timestamp, pose);

                if (!hasStartedFollowing && wantReset) {
                    zeroSensors(startingPosition);
                    System.out.println("Position reset for auto");
                    hasStartedFollowing = true;
                    wantReset = false;
                }

                //                                System.out.println("DRIVE VECTOR" + driveVector);

                mPeriodicIO.forward = driveVector.x();
                mPeriodicIO.strafe = driveVector.y();
                mPeriodicIO.rotation = 0;

                double rotationInput = Util.deadBand(
                    Util.limit(
                        rotationCorrection * rotationScalar * driveVector.norm(),
                        motionPlanner.getMaxRotationSpeed()
                    ),
                    0.01
                );

                mPeriodicIO.error = motionPlanner.error();
                mPeriodicIO.path_setpoint = motionPlanner.setpoint();
                mPeriodicIO.drive_vector = driveVector;
                if (!mOverrideTrajectory) {
                    //                    System.out.println("ROTATIONINPUT==" + rotationInput);
                    setVelocity(
                        SwerveKinematics.updateDriveVectors(
                            driveVector,
                            rotationInput,
                            pose,
                            robotCentric
                        )
                    );
                }
            } else {
                setVelocity(ZERO_DRIVE_VECTOR);
                if (alwaysConfigureModules) requireModuleConfiguration();
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");
        resetPigeon();
        setHeading(pose.getRotation());
        resetPose(pose);

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
        SmartDashboard.putBoolean(
            "Drive/TeleopFieldCentric",
            this.mPeriodicIO.field_relative
        );
        SmartDashboard
            .getEntry("Drive/TeleopFieldCentric")
            .addListener(
                notification -> {
                    this.mPeriodicIO.field_relative = notification.value.getBoolean();
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
    }
}
