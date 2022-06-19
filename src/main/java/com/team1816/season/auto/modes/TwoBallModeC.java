package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;

public class TwoBallModeC extends AutoModeBase {

    public TwoBallModeC() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_C,
                TrajectorySet.TWO_BALL_C_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball C Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.NORTH - 5), // to be changed
                    new CollectAction(true),
                    new RampUpShooterAction(Shooter.MID_VELOCITY) //8650
                ),
                trajectory,
                new ShootAction(true, true),
                new WaitAction(4)
            )
        );
        runAction(new StopAction(false));
    }
}
