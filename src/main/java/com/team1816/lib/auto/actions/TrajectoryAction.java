package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.Drive;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TrajectoryAction implements Action {

    @Inject
    private static Drive.Factory mDriveFactory;

    private final RamseteCommand mCommand;
    private final Trajectory mTrajectory;
    private final Drive mDrive;

    public TrajectoryAction(Trajectory trajectory) {
        mDrive = mDriveFactory.getInstance();
        mTrajectory = trajectory;
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
