package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;

public class TwoBallModeA extends AutoModeBase {

    public TwoBallModeA() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_A,
                TrajectorySet.TWO_BALL_A_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball A Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.CARDINAL_NORTH + 15), // to be changed
                    new CollectAction(true),
                    new RampUpShooterAction(13000) // make actual shooting vel
                ),
                trajectory,
                new ShootAction(true, true),
                new WaitAction(2)
            )
        );
        runAction(new StopAction(false));
    }
}
