package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;

public class DriveStraightShootMode extends AutoModeBase {

    public DriveStraightShootMode() {
        trajectory = new TrajectoryAction(TrajectorySet.TUNE_DRIVETRAIN);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new DriveOpenLoopAction(2, .25),
                new RampUpShooterAction(Shooter.COAST_VELOCITY), // make actual shooting vel
                new ShootAction(true, true),
                new WaitAction(2),
                new StopAction(false)
            )
        );
    }
}
