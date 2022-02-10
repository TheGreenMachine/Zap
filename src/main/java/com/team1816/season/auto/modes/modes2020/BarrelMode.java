package com.team1816.season.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team254.lib.trajectory.Trajectory;

public class BarrelMode extends AutoModeBase {

    public BarrelMode() {
        trajectory = new TrajectoryAction(TrajectorySet.BARREL, TrajectorySet.BARREL_HEADINGS);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(trajectory);
    }

}
