package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;

public class AutoAimAndRev extends ParallelAction {

    public AutoAimAndRev(double visionDuration, int defaultShootVel) {
        super(
            new AutoAimAction(visionDuration),
            new SeriesAction(
                new WaitAction(visionDuration - 1.2),
                new RampUpShooterAction(defaultShootVel)
            )
        );
    }
}
