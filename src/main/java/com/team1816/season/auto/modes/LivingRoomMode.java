package com.team1816.season.auto.modes;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.paths.LivingRoomPath;
import java.util.List;

public class LivingRoomMode extends AutoMode {

    public LivingRoomMode() {
        super(List.of(new TrajectoryAction(new LivingRoomPath())));
    }

    @Override
    protected void routine() {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(new SeriesAction(new CollectAction(true), trajectoryActions.get(0)));
    }
}
