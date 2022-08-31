package com.team1816.lib.auto.actions;

import static com.team1816.lib.subsystems.drive.Drive.*;
import static com.team1816.lib.subsystems.drive.SwerveDrive.swerveKinematics;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
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

    private final Command command;
    private final Trajectory trajectory;
    private final List<Rotation2d> headings;
    private final Drive drive;

    public TrajectoryAction(AutoPath autoPath) {
        this(autoPath.getAsTrajectory(), autoPath.getAsTrajectoryHeadings());
    }

    public TrajectoryAction(Trajectory trajectory, List<Rotation2d> headings) {
        drive = Injector.get(Drive.Factory.class).getInstance();
        this.trajectory = trajectory;
        this.headings = headings;

        // create command (wpi version of an action)
        if (drive instanceof TankDrive) {
            command =
                new RamseteCommand(
                    trajectory,
                    drive::getPose,
                    new RamseteController(), //defaults of
                    new DifferentialDriveKinematics(
                        Units.inchesToMeters(kDriveWheelTrackWidthInches)
                    ),
                    ((TankDrive) drive)::updateTrajectoryVelocities
                );
        } else if (drive instanceof SwerveDrive) {
            var thetaController = new ProfiledPIDController(
                kPThetaController,
                0,
                0,
                kThetaControllerConstraints
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            command =
                new SwerveControllerCommand(
                    trajectory,
                    drive::getPose,
                    swerveKinematics,
                    new PIDController(kPXController, 0, 0),
                    new PIDController(kPYController, 0, 0),
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
            "Starting trajectory! (seconds = " + trajectory.getTotalTimeSeconds() + ")"
        );
        drive.startTrajectory(trajectory, headings);
        command.initialize();
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}
