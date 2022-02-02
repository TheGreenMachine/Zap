package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.SwerveDrive;
import com.team1816.season.subsystems.SwerveModule;
import com.team1816.season.subsystems.TankDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TrajectoryAction implements Action {

    @Inject
    private static Drive.Factory mDriveFactory;

    private final Command mCommand;
    private final Trajectory mTrajectory;
    private final Drive mDrive;

    public TrajectoryAction(Trajectory trajectory) {
        mDrive = mDriveFactory.getInstance();
        mTrajectory = trajectory;
        if(mDrive instanceof TankDrive){
            mCommand =
                new RamseteCommand(
                    trajectory,
                    mDrive::getPose,
                    new RamseteController(),
                    new DifferentialDriveKinematics(
                        Units.inchesToMeters(Constants.kDriveWheelTrackWidthInches)
                    ),
                    mDrive::updateTrajectoryVelocities
                );
        } else if(mDrive instanceof SwerveDrive){
            var thetaController =
                new ProfiledPIDController(
                    Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            mCommand =
                new SwerveControllerCommand(
                    trajectory,
                    mDrive::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.kPXController, 0, 0),
                    new PIDController(Constants.kPYController, 0, 0),
                    thetaController,
                    mDrive::getDesiredRotation2d,
                    mDrive::setModuleStates
                );
        } else {
            System.out.println(" oh man oh god I'm neither swerve nor tank! " + mDrive.toString());
            mCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        return mCommand.isFinished();
    }

    @Override
    public void update() {
        mCommand.execute();
    }

    @Override
    public void done() {
        mCommand.end(false);
        System.out.println("Trajectory finished");
    }

    @Override
    public void start() {
        System.out.println(
            "Starting trajectory! (seconds=" + mTrajectory.getTotalTimeSeconds() + ")"
        );
        mDrive.startTrajectory(mTrajectory);
        mCommand.initialize();
    }

    public Trajectory getTrajectory() {
        return mTrajectory;
    }
}
