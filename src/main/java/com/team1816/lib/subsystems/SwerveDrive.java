package com.team1816.lib.subsystems;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.season.Constants;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SwerveDriveHelper;
import com.team254.lib.util.SwerveDriveSignal;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    public static final String NAME = "drivetrain";

    // Components
    public SwerveModule[] swerveModules;

    // Trajectory
    protected List<Rotation2d> headingsList;
    protected int trajectoryIndex = 0;

    // Odometry variables
    private SwerveDriveOdometry swerveOdometry;
    private final SwerveDriveHelper swerveDriveHelper = new SwerveDriveHelper();

    // States

    public SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
    SwerveModuleState[] actualModuleStates = new SwerveModuleState[4];
    public double[] motorTemperatures = new double[4];

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

        swerveOdometry =
            new SwerveDriveOdometry(
                Constants.Swerve.swerveKinematics,
                getActualHeading()
            );
    }

    // autonomous (Trajectory_Following) loop is in setModuleStates
    @Override
    public synchronized void writeToHardware() {
        if (controlState == ControlState.OPEN_LOOP) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredModuleStates,
                Constants.kOpenLoopMaxVelMeters
            );
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(desiredModuleStates[i], true);
            }
        }
    }

    @Override
    public synchronized void readFromHardware() {
        for (int i = 0; i < 4; i++) {
            // logging actual angle and velocity of swerve motors (azimuth & drive)
            actualModuleStates[i] = swerveModules[i].getActualState();

            // logging current temperatures of each module's drive motor
            motorTemperatures[i] = swerveModules[i].getMotorTemp();
        }
        chassisSpeed =
            Constants.Swerve.swerveKinematics.toChassisSpeeds(actualModuleStates);

        if (RobotBase.isSimulation()) {
            simulateGyroOffset();
        }

        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());

        swerveOdometry.update(actualHeading, actualModuleStates);
        updateRobotState();
    }

    public Rotation2d getTrajectoryHeadings() {
        if (headingsList == null) {
            System.out.println("headings are empty!");
            return Constants.EmptyRotation;
        } else if (trajectoryIndex > headingsList.size() - 1) {
            //System.out.println("heck the headings aren't long enough");
            return Constants.EmptyRotation;
        }
        if (
            getTrajectoryTimestamp() >
            trajectory.getStates().get(trajectoryIndex).timeSeconds ||
            trajectoryIndex == 0
        ) trajectoryIndex++;
        if (trajectoryIndex >= headingsList.size()) {
            System.out.println(headingsList.get(headingsList.size() - 1) + " = max");
            return headingsList.get(headingsList.size() - 1);
        }
        double timeBetweenPoints =
            (
                trajectory.getStates().get(trajectoryIndex).timeSeconds -
                trajectory.getStates().get(trajectoryIndex - 1).timeSeconds
            );
        Rotation2d heading;
        heading =
            headingsList
                .get(trajectoryIndex - 1)
                .interpolate(
                    headingsList.get(trajectoryIndex),
                    getTrajectoryTimestamp() / timeBetweenPoints
                );
        return heading;
    }

    // autonomous (trajectory following)
    @Override
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        super.startTrajectory(trajectory, headings);
        headingsList = headings;
        trajectoryIndex = 0;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (controlState != ControlState.TRAJECTORY_FOLLOWING) {
            controlState = ControlState.TRAJECTORY_FOLLOWING;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            (Constants.kPathFollowingMaxVelMeters)
        );
        desiredModuleStates = desiredStates;
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], false);
        }
    }

    @Override
    public void updateRobotState() {
        robotState.fieldToVehicle = swerveOdometry.getPoseMeters();
        robotState.deltaVehicle =
            new ChassisSpeeds(
                chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond,
                chassisSpeed.omegaRadiansPerSecond
            );
        // check if motors are overheating - update robotState
        SmartDashboard.putNumber("Drive/Temperature", motorTemperatures[0]);
        if (!robotState.overheating) {
            for (int i = 0; i < 4; i++) {
                if (motorTemperatures[i] > heatThreshold) {
                    robotState.overheating = true;
                    break;
                }
            }
        } else {
            for (int i = 0; i < 4; i++) {
                if (motorTemperatures[i] < heatThreshold - 10) { // TODO make yml or other const for point where overheating is no longer true
                    robotState.overheating = false;
                    break;
                }
            }
        }
    }

    // general setters
    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            controlState = ControlState.OPEN_LOOP;
        }
        SwerveModuleState[] desiredStatesSignal = new SwerveModuleState[4];
        for (int i = 0; i < desiredStatesSignal.length; i++) {
            desiredStatesSignal[i] =
                new SwerveModuleState(
                    ((SwerveDriveSignal) signal).getWheelSpeeds()[i],
                    ((SwerveDriveSignal) signal).getWheelAzimuths()[i]
                );
        }

        desiredModuleStates = desiredStatesSignal;
    }

    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        SwerveDriveSignal signal = swerveDriveHelper.calculateDriveSignal(
            forward,
            strafe,
            rotation,
            isSlowMode,
            true,
            false
        );

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(signal);
        }
    }

    @Override
    public synchronized void setBraking(boolean braking) {
        isBraking = braking;
        if (braking) {
            setOpenLoop(SwerveDriveSignal.BRAKE);
        }
    }

    // general getters
    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public SwerveModuleState[] getStates() {
        return Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeed);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());
        swerveOdometry.resetPosition(pose, actualHeading);
        swerveOdometry.update(actualHeading, actualModuleStates);
        updateRobotState();
    }

    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");

        // resetting ACTUAL module states
        for (int i = 0; i < 4; i++) {
            actualModuleStates[i] = new SwerveModuleState();
        }

        resetOdometry(pose);
        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    @Override
    public synchronized void stop() {
        for (int i = 0; i < 4; i++) {
            SwerveModuleState stoppedState = new SwerveModuleState(
                0,
                swerveModules[i].getActualState().angle
            );
            swerveModules[i].setDesiredState(
                    stoppedState,
                    controlState == ControlState.OPEN_LOOP
                );
            desiredModuleStates[i] = stoppedState;
        }
    }

    // other
    @Override
    public boolean checkSystem() {
        boolean modulesPassed = true;
        for (SwerveModule swerveModule : swerveModules) {
            if (!swerveModule.checkSystem()) {
                modulesPassed = false;
                break;
            }
        }

        return modulesPassed;
    }

    // getters
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        defaultPIDConfig.kI = 0.0;
        defaultPIDConfig.kD = 0.0;
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault("slot0", defaultPIDConfig)
            : defaultPIDConfig;
    }
}
