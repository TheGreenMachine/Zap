package com.team1816.lib.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.season.Constants;
import com.team1816.season.auto.AutoModeSelector;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveHelper;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    public static final String NAME = "drivetrain";

    private final SwerveDriveHelper swerveDriveHelper = new SwerveDriveHelper();

    public SwerveModule[] swerveModules;

    @Inject
    private static AutoModeSelector autoModeSelector;

    // Odometry variables
    private SwerveDriveOdometry swerveOdometry;

    public SwerveDrive() {
        super();
        swerveModules = new SwerveModule[4];

        // enableDigital all Talons in open loop mode
        swerveModules[Constants.Swerve.kFrontLeft] =
            factory.getSwerveModule(NAME, "frontLeft");
        swerveModules[Constants.Swerve.kFrontRight] =
            factory.getSwerveModule(NAME, "frontRight");
        swerveModules[Constants.Swerve.kBackLeft] =
            factory.getSwerveModule(NAME, "backLeft");
        swerveModules[Constants.Swerve.kBackRight] =
            factory.getSwerveModule(NAME, "backRight");

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = false;
        setBrakeMode(mIsBrakeMode);

        swerveOdometry =
            new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getHeading());
    }

    // autonomous (Trajectory_Following) loop is in setModuleStates
    @Override
    public synchronized void writeToHardware() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                mPeriodicIO.desiredModuleStates,
                Constants.kOpenLoopMaxVelMeters
            ); // TODO get swerve max speed in meters/s
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(
                        mPeriodicIO.desiredModuleStates[i],
                        !mIsBrakeMode
                    );
            }
        }
    }

    @Override
    public synchronized void readFromHardware() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        mPeriodicIO.chassisSpeed =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(states);

        if (RobotBase.isSimulation()) { // calculate rotation based on actualModeStates
            // simulates rotation by computing the rotational motion per interval
            mPeriodicIO.gyro_heading_no_offset =
                mPeriodicIO.gyro_heading_no_offset.rotateBy(
                    new Rotation2d(
                        mPeriodicIO.chassisSpeed.omegaRadiansPerSecond *
                        Constants.kLooperDt *
                        0.01
                    )
                );
            // calculate rotation with gyro drift
            gyroDrift -= 0;
            mPeriodicIO.gyro_heading_no_offset =
                mPeriodicIO.gyro_heading_no_offset.rotateBy(
                    Rotation2d.fromDegrees(gyroDrift)
                );
        } else {
            mPeriodicIO.gyro_heading_no_offset =
                Rotation2d.fromDegrees(mInfrastructure.getYaw());
        }

        mPeriodicIO.gyro_heading = mPeriodicIO.gyro_heading_no_offset;
        swerveOdometry.update(mPeriodicIO.gyro_heading, states);
        updateRobotState();
    }

    public Rotation2d getTrajectoryHeadings() {
        if (mHeadings == null) {
            System.out.println("headings are empty!");
            return Constants.EmptyRotation;
        } else if (mTrajectoryIndex > mHeadings.size() - 1) {
            //System.out.println("heck the headings aren't long enough");
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
        return heading;
    }

    // autonomous (trajectory following)
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        System.out.println("STARTING TRAJECTORY ");
        mPeriodicIO.timestamp = 0;
        mTrajectoryStart = 0;
        mTrajectory = trajectory;
        mHeadings = headings;
        //        if (!trajectoryStarted) {
        //            zeroSensors(trajectory.getInitialPose());
        //            trajectoryStarted = true; // massive hack here woo
        //        }
        mTrajectoryIndex = 0;
        updateRobotState();
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        mOverrideTrajectory = false;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        mDriveControlState = DriveControlState.TRAJECTORY_FOLLOWING;
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            (Constants.kPathFollowingMaxVelMeters)
        );
        mPeriodicIO.desiredModuleStates = desiredStates;
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], false);
        }
    }

    private void updateRobotState() {
        //        System.out.println(
        //            mInfrastructure.getYaw() +
        //            " = robot pigeon - diff = " +
        //            (
        //                mInfrastructure.getYaw() -
        //                robotState.field_to_vehicle.getRotation().getDegrees()
        //            )
        //        );
        robotState.field_to_vehicle = swerveOdometry.getPoseMeters();
        robotState.chassis_speeds =
            new ChassisSpeeds(
                mPeriodicIO.chassisSpeed.vxMetersPerSecond,
                mPeriodicIO.chassisSpeed.vyMetersPerSecond,
                mPeriodicIO.chassisSpeed.omegaRadiansPerSecond
            );
        robotState.delta_field_to_vehicle =
            new Twist2d(
                // these three may be missing conversions from velocity to change in pose? (meters/s to x-y-theta/updateTime)
                // not sure because field_to_vehicle is also being plugged directly into field as a value in meters
                mPeriodicIO.chassisSpeed.vxMetersPerSecond * Constants.kLooperDt,
                mPeriodicIO.chassisSpeed.vyMetersPerSecond * Constants.kLooperDt,
                mPeriodicIO.chassisSpeed.omegaRadiansPerSecond * Constants.kLooperDt
            );
    }

    @Override
    protected void updateOpenLoopPeriodic() {
        // no openLoop update needed
    }

    // general setters
    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < states.length; i++) {
            states[i] =
                new SwerveModuleState(
                    ((SwerveDriveSignal) signal).getWheelSpeeds()[i],
                    ((SwerveDriveSignal) signal).getWheelAzimuths()[i]
                );
        }

        mPeriodicIO.desiredModuleStates = states;
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
        SwerveDriveSignal signal = swerveDriveHelper.calculateDriveSignal(
            forward,
            strafe,
            rotation,
            low_power,
            true,
            use_heading_controller
        );

        setOpenLoop(signal);
    }

    @Override
    public synchronized void setBrakeMode(boolean on) {
        super.setBrakeMode(on);
        for (int i = 0; i < swerveModules.length; i++) {
            if (on) {
                setOpenLoop(SwerveDriveSignal.BRAKE);
            }
            swerveModules[i].setDriveBrakeMode(on);
        }
    }

    // general getters
    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public SwerveModuleState[] getStates() {
        return Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            mPeriodicIO.chassisSpeed
        );
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getHeading());
    }

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");
        setBrakeMode(false);
        //        mInfrastructure.resetPigeon(pose.getRotation());
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mInfrastructure.getYaw());
        resetOdometry(pose);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(new SwerveModuleState(), true);
            mPeriodicIO.desiredModuleStates[i] = new SwerveModuleState();
            mPeriodicIO.chassisSpeed = new ChassisSpeeds();
        }
        autoModeSelector.setHardwareFailure(false);
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
        for (SwerveModule swerveModule : swerveModules) {
            if (!swerveModule.checkSystem()) {
                modulesPassed = false;
                break;
            }
        }

        return modulesPassed; // not actually doing anything
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

    // getters
    @Override
    public double getKP() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault(pidSlot, defaultPIDConfig)
                .kP
            : 0.0;
    }

    @Override
    public double getKI() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kI = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault(pidSlot, defaultPIDConfig)
                .kI
            : 0.0;
    }

    @Override
    public double getKD() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kD = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault(pidSlot, defaultPIDConfig)
                .kD
            : 0.0;
    }

    @Override
    public double getKF() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault(pidSlot, defaultPIDConfig)
                .kF
            : 0.0;
    }
}
