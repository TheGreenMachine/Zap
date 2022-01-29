package com.team1816.frcSeason.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frcSeason.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.modes.AutoModeBase;

public class BarrelMode extends AutoModeBase {

    private DriveTrajectory trajectory;

    @Inject
    private static TrajectorySet trajectorySet;

    public BarrelMode() {
        this.trajectory = new DriveTrajectory(trajectorySet.BARREL, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(trajectory);
    }
}
