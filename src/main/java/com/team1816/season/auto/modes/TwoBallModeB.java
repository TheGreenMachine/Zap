package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;

public class TwoBallModeB extends AutoModeBase {

    public TwoBallModeB() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_B,
                TrajectorySet.TWO_BALL_B_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball B Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.CARDINAL_NORTH + 25), // to be changed
                    new CollectAction(true)
                ),
                trajectory,
                new AutoAimAndRev(2, 11000),
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
