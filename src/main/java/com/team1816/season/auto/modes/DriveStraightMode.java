package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.RampUpShooterAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode() {
        trajectory = new TrajectoryAction(TrajectorySet.DRIVE_STRAIGHT);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(trajectory);
        new SeriesAction(
            new RampUpShooterAction(Shooter.COAST_VELOCITY), // make actual shooting vel
            new WaitUntilInsideRegion(
                new Translation2d(248, 194),
                new Translation2d(275, 171)
            ),
            new ShootAction(true, true),
            new WaitAction(2),
            new ParallelAction( // stop all at end - make a stop action in the future
                new CollectAction(false),
                new RampUpShooterAction(Shooter.COAST_VELOCITY),
                new ShootAction(false, false)
            )
        );
    }
}
