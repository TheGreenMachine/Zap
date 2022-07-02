package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TwoBallPathB;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;

public class TwoBallModeB extends AutoMode {

    public TwoBallModeB() {
        trajectoryAction = new TrajectoryAction(new TwoBallPathB());
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball B Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.NORTH + 5), // to be changed
                    new CollectAction(true),
                    new RampUpShooterAction(Shooter.MID_VELOCITY)
                ),
                trajectoryAction,
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
