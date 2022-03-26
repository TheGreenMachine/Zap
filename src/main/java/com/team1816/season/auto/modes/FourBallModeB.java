package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;

public class FourBallModeB extends AutoModeBase {

    private final TrajectoryAction trajectory1;
    private final TrajectoryAction trajectory2;

    public FourBallModeB() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_B,
                TrajectorySet.TWO_BALL_B_HEADINGS
            );
        trajectory1 =
            new TrajectoryAction(
                TrajectorySet.FOUR_BALL_B2,
                TrajectorySet.FOUR_BALL_B2_HEADINGS
            );
        trajectory2 =
            new TrajectoryAction(
                TrajectorySet.FOUR_BALL_B3,
                TrajectorySet.FOUR_BALL_B3_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball B Mode");
        runAction(new WaitAction(1));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.CARDINAL_NORTH + 25), // to be changed
                    new CollectAction(true),
                    new RampUpShooterAction(13000) // make actual shooting vel
                ),
                trajectory,
                new ShootAction(true, true),
                new WaitAction(3),
                new ShootAction(false, true),
                trajectory1,
                new WaitAction(3),
                new TurretAction(Turret.CARDINAL_SOUTH + 25),
                trajectory2,
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
