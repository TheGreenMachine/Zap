package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
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
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.NORTH + 5), // to be changed
                    new CollectAction(true)
                ),
                new WaitAction(0.5),
                trajectory,
                new RampUpShooterAction(8650),
                new WaitAction(0.75),
                new ShootAction(true, true),
                new WaitAction(1.5),
                new ShootAction(false, true),
                trajectory1,
                new WaitAction(1.25),
                new TurretAction(Turret.SOUTH + 45), // 32
                trajectory2,
                //                new AutoAimAction(1.5),
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
