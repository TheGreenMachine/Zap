package com.team1816.frcSeason.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frcSeason.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class DriveStraightMode extends AutoModeBase {

    private DriveTrajectory mDriveStraight;
    @Inject
    private static TrajectorySet trajectorySet;

    public DriveStraightMode() {
        mDriveStraight = new DriveTrajectory(trajectorySet.DRIVE_STRAIGHT, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(mDriveStraight);
    }
}
