package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.frcSeason.RobotState;
import com.team1816.frcSeason.subsystems.Drive;
import com.team1816.frcSeason.subsystems.SwerveDrive;
import com.team1816.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectory implements Action {

    @Inject
    private static Drive.Factory mDriveFactory;

    @Inject
    private static RobotState mRobotState;

    private static Drive mDrive;

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final Rotation2d targetHeading;
    private final boolean mResetPose;
    private boolean done;

    public DriveTrajectory(
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
        Rotation2d targetHeading,
        boolean resetPose
    ) {
        mDrive = mDriveFactory.getInstance();
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        this.targetHeading = targetHeading;
        mResetPose = resetPose;
    }

    public DriveTrajectory(
        Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
        boolean resetPose
    ) {
        this(trajectory, Rotation2d.identity(), resetPose);
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            if (!done) {
                System.out.println("Trajectory finished");
            }
            done = true;
            return true;
        }
        return false;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        System.out.println(
            "Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")"
        );
        var pose = mTrajectory.getState().state().getPose();

        if (mResetPose) {
            mDrive.setHeading(pose.getRotation());
            mRobotState.reset(Timer.getFPGATimestamp(), pose);
            if (mDrive instanceof SwerveDrive) {
                ((SwerveDrive) mDrive).setStartingPose(pose);
                ((SwerveDrive) mDrive).setWantReset(true);
            }
        }
        mDrive.setTrajectory(mTrajectory, targetHeading);
    }
}
