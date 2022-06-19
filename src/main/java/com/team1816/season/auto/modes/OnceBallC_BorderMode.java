package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;

public class OnceBallC_BorderMode extends AutoModeBase {

    public OnceBallC_BorderMode() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.ONE_BALL_C_BORDER,
                TrajectorySet.ONE_BALL_C_BORDER_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball B Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.NORTH - 4), // to be changed
                    new RampUpShooterAction(Shooter.MID_VELOCITY)
                ),
                new WaitAction(6),
                trajectory,
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
