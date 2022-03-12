package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.RampUpShooterAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.auto.actions.TurretAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Shooter;
import com.team1816.season.subsystems.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TwoBallModeA extends AutoModeBase {
    public Pose2d startingPose;
    private TrajectoryAction trajectory1;

    public TwoBallModeA() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_A,
                TrajectorySet.TWO_BALL_A_HEADINGS
            );
        trajectory1 =
            new TrajectoryAction(
                TrajectorySet.TWO_BALL_A,
                TrajectorySet.TWO_BALL_A_HEADINGS
            );
        startingPose = trajectory.getTrajectory().getInitialPose();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Ball A Mode");
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new RampUpShooterAction(Shooter.NEAR_VELOCITY), // make actual shooting vel
                new WaitAction(1),
                new ShootAction(true, false),
                trajectory,
                new ParallelAction(
                    new SeriesAction(
                        new WaitAction(0.5),
                        new WaitUntilInsideRegion(
                            new Translation2d(215, 239),
                            new Translation2d(233, 211)
                        ),
                        new CollectAction(false),
                        new WaitUntilInsideRegion(
                            new Translation2d(262, 199), // make actual region to change hood
                            new Translation2d(275, 168)
                        ),
                        new ShootAction(true, false),
                        new WaitAction(2)
                    ),
                    trajectory1
                )
            )
        );
    }
}
