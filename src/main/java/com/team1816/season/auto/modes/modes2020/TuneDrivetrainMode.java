package com.team1816.season.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class TuneDrivetrainMode extends AutoModeBase {

    public TuneDrivetrainMode() {
        trajectory = new TrajectoryAction(TrajectorySet.TUNE_DRIVETRAIN);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(trajectory);
    }
}
