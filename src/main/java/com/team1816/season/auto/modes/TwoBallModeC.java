package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TwoBallPathC;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;
import java.util.List;

public class TwoBallModeC extends AutoMode {

    public TwoBallModeC() {
        super(List.of(new TrajectoryAction(new TwoBallPathC())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball C Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.kNorth - 5), // to be changed
                    new CollectAction(true),
                    new RampUpShooterAction(Shooter.MID_VELOCITY) //8650
                ),
                trajectoryActions.get(0),
                new ShootAction(true, true),
                new WaitAction(4)
            )
        );
        runAction(new StopAction(false));
    }
}
