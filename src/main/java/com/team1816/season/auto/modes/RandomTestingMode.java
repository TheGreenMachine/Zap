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

public class RandomTestingMode extends AutoModeBase {

    public Pose2d startingPose;
    private TrajectoryAction trajectory1;
    private TrajectoryAction trajectory2;

    public RandomTestingMode() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_A,
                TrajectorySet.FIVE_BALL_A_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Five Ball Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new RampUpShooterAction(Shooter.MID_VELOCITY), // make actual shooting vel
                trajectory,
                new ParallelAction(
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(210, 180)
                        )
                    )
                )
            )
        );
    }
}
