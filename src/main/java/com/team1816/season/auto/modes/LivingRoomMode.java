package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.paths.TrajectorySet;

public class LivingRoomMode extends AutoModeBase {

    public LivingRoomMode() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.LIVING_ROOM,
                TrajectorySet.LIVING_ROOM_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(new SeriesAction(new CollectAction(true), trajectory));
    }
}
