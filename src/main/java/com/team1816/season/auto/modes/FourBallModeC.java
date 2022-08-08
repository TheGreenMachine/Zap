package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.FourBallPathC2;
import com.team1816.season.auto.paths.FourBallPathC3;
import com.team1816.season.auto.paths.TwoBallPathC;
import com.team1816.season.subsystems.Turret;
import java.util.List;

public class FourBallModeC extends AutoMode {

    public FourBallModeC() {
        super(
            List.of(
                new TrajectoryAction(new TwoBallPathC()),
                new TrajectoryAction(new FourBallPathC2()),
                new TrajectoryAction(new FourBallPathC3())
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball C Mode");
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new TurretAction(Turret.kNorth - 10), // to be changed
                    new CollectAction(true)
                ),
                trajectoryActions.get(0),
                new AutoAimAndRev(1.7, 11000),
                new ShootAction(true, true),
                new WaitAction(1.75),
                new ShootAction(false, true),
                trajectoryActions.get(1),
                new WaitAction(1),
                new TurretAction(Turret.kSouth), // this is in dead zone so tune heading or turret angle
                trajectoryActions.get(2),
                new AutoAimAndRev(1, 11000),
                new ShootAction(true, true),
                new WaitAction(3)
            )
        );
        runAction(new StopAction(false));
    }
}
