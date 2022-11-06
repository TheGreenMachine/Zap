package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TwoBallPathA;
import com.team1816.season.subsystems.Shooter;
import java.util.List;

public class TwoBallModeA extends AutoMode {

    public TwoBallModeA() {
        super(List.of(new TrajectoryAction(new TwoBallPathA())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball A Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    //new TurretAction(Turret.kNorth + 5), // to be changed
                    new AbsoluteTurretAction(),
                    new CollectAction(true),
                    new RampUpShooterAction(Shooter.MID_VELOCITY)
                ),
                trajectoryActions.get(0),
                new ShootAction(true, true),
                new WaitAction(4)
            )
        );
        runAction(new StopAction(false));
    }
}
