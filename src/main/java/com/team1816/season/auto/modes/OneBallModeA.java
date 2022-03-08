package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.RampUpShooterAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;

public class OneBallModeA extends AutoModeBase {
    public Pose2d startingPose;

    public OneBallModeA() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.ONE_BALL_A,
                TrajectorySet.ONE_BALL_A_HEADINGS
            );
        startingPose = trajectory.getTrajectory().getInitialPose();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball C Mode");
        runAction(
            new SeriesAction(
                new WaitAction(.5),
                new RampUpShooterAction(Shooter.NEAR_VELOCITY),
                new WaitAction(2),
                new ShootAction(true, false),
                new WaitAction(1),
                trajectory,
                new ParallelAction( // stop all at end - make a stop action in the future
                    new CollectAction(false),
                    new RampUpShooterAction(Shooter.COAST_VELOCITY),
                    new ShootAction(false, false)
                )
            )
        );
    }
}
