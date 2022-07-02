package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.FourBallPathC2;
import com.team1816.season.auto.paths.FourBallPathC3;
import com.team1816.season.auto.paths.TwoBallPathC;
import com.team1816.season.subsystems.Turret;

public class FourBallModeC extends AutoMode {

    private final TrajectoryAction trajectory1;
    private final TrajectoryAction trajectory2;

    public FourBallModeC() {
        trajectoryAction = new TrajectoryAction(new TwoBallPathC());
        trajectory1 = new TrajectoryAction(new FourBallPathC2());
        trajectory2 = new TrajectoryAction(new FourBallPathC3());
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
                trajectoryAction,
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
