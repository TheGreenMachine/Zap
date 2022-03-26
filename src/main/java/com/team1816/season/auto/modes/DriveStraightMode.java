package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.DriveOpenLoopAction;
import com.team1816.season.auto.paths.TrajectorySet;

public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode() {
        trajectory = new TrajectoryAction(TrajectorySet.TUNE_DRIVETRAIN);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(new DriveOpenLoopAction(2, .25));
    }
}
