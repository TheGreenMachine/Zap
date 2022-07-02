package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.FourBallPathB2;
import com.team1816.season.auto.paths.FourBallPathB3;
import com.team1816.season.auto.paths.TwoBallPathB;
import com.team1816.season.subsystems.Turret;

public class FourBallModeB extends AutoMode {

    private final TrajectoryAction trajectory1;
    private final TrajectoryAction trajectory2;

    public FourBallModeB() {
        trajectoryAction = new TrajectoryAction(new TwoBallPathB());
        trajectory1 = new TrajectoryAction(new FourBallPathB2());
        trajectory2 = new TrajectoryAction(new FourBallPathB3());
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
                trajectoryAction,
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
