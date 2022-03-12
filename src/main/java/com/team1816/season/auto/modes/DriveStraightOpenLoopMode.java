package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.DriveOpenLoopAction;
import com.team1816.season.paths.TrajectorySet;

public class DriveStraightOpenLoopMode extends AutoModeBase {

    public DriveStraightOpenLoopMode() { // trajectory is a dummy value just for the starting pose
        trajectory =
            new TrajectoryAction(
                TrajectorySet.LIVING_ROOM,
                TrajectorySet.LIVING_ROOM_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(.5));
        runAction(new DriveOpenLoopAction(2, .25));
    }
}
