package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.SwerveDrive;
import com.team1816.lib.subsystems.TankDrive;
import com.team1816.season.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

public class TrajectoryAction implements Action {

    @Inject
    private static Drive.Factory driveFactory;

    private final Command command;
    private final Trajectory trajectory;
    private final List<Rotation2d> headings;
    private final Drive drive;

    public TrajectoryAction(Trajectory trajectory) {
        this(trajectory, null);
    }

    public TrajectoryAction(Trajectory trajectory, List<Rotation2d> headings) {
        drive = driveFactory.getInstance();
        this.trajectory = trajectory;
        this.headings = headings;
        if (drive instanceof TankDrive) {
            command =
                new RamseteCommand(
                    trajectory,
                    drive::getPose,
                    new RamseteController(), //defaults of
                    new DifferentialDriveKinematics(
                        Units.inchesToMeters(Constants.kDriveWheelTrackWidthInches)
                    ),
                    ((TankDrive) drive)::updateTrajectoryVelocities
                );
        } else if (drive instanceof SwerveDrive) {
            var thetaController = new ProfiledPIDController(
                Constants.kPThetaController,
                0,
                0,
                Constants.kThetaControllerConstraints
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            command =
                new SwerveControllerCommand(
                    trajectory,
                    drive::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.kPXController, 0, 0),
                    new PIDController(Constants.kPYController, 0, 0),
                    thetaController,
                    ((SwerveDrive) drive)::getTrajectoryHeadings,
                    ((SwerveDrive) drive)::setModuleStates
                );
        } else {
            System.out.println(
                " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            command = null;
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void update() {
        command.execute();
    }

    @Override
    public void done() {
        command.end(false);
        drive.stop();
    }

    @Override
    public void start() {
        System.out.println(
            "Starting trajectory! (seconds=" + trajectory.getTotalTimeSeconds() + ")"
        );
        drive.startTrajectory(trajectory, headings);
        command.initialize();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}
