package com.team1816.frcSeason.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frcSeason.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.geometry.Rotation2d;

public class TuneDrivetrainMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    @Inject
    private static TrajectorySet trajectorySet;

    public TuneDrivetrainMode(boolean isReversed) {
        var traj = isReversed
            ? trajectorySet.TUNE_DRIVETRAIN_REVERSE
            : trajectorySet.TUNE_DRIVETRAIN;
        trajectory = new DriveTrajectory(traj, Rotation2d.fromDegrees(90), true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Tune Drivetrain path");
        runAction(new WaitAction(1));
        runAction(trajectory);
    }
}
