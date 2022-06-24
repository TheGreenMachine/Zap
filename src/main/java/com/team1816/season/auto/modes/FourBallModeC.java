package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;

public class FourBallModeC extends AutoModeBase {

    private final TrajectoryAction trajectory1;
    private final TrajectoryAction trajectory2;

    public FourBallModeC() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_C,
                TrajectorySet.TWO_BALL_C_HEADINGS
            );
        trajectory1 =
            new TrajectoryAction(
                TrajectorySet.FOUR_BALL_C2,
                TrajectorySet.FOUR_BALL_C2_HEADINGS
            );
        trajectory2 =
            new TrajectoryAction(
                TrajectorySet.FOUR_BALL_C3,
                TrajectorySet.FOUR_BALL_C3_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball C Mode");
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.NORTH - 10), // to be changed
                    new CollectAction(true)
                ),
                trajectory,
                new AutoAimAndRev(1.7, 11000),
                new ShootAction(true, true),
                new WaitAction(1.75),
                new ShootAction(false, true),
                trajectory1,
                new WaitAction(1),
                new TurretAction(Turret.SOUTH), // this is in dead zone so tune heading or turret angle
                trajectory2,
                new AutoAimAndRev(1, 11000),
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
