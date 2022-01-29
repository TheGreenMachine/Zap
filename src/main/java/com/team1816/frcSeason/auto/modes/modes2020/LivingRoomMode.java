package com.team1816.frcSeason.auto.modes.modes2020;

import com.google.inject.Inject;
import com.team1816.frcSeason.paths.TrajectorySet;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveTrajectory;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

public class LivingRoomMode extends AutoModeBase {

    private DriveTrajectory mDriveTrajectory;

    @Inject
    private static TrajectorySet trajectorySet;

    public LivingRoomMode() {
        var trajectory = trajectorySet.LIVING_ROOM;
        mDriveTrajectory = new DriveTrajectory(trajectory, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(mDriveTrajectory);
    }
}
