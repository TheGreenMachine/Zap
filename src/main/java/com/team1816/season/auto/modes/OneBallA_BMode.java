package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.OneBallA_BPath;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;
import java.util.List;

public class OneBallA_BMode extends AutoMode {

    public OneBallA_BMode() {
        super(List.of(new TrajectoryAction(new OneBallA_BPath())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball B Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.kNorth + 2), // to be changed
                    new RampUpShooterAction(Shooter.MID_VELOCITY),
                    new WaitAction(10)
                ),
                trajectoryActions.get(0),
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
