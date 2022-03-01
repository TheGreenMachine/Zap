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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ThreeBallModeC extends AutoModeBase {
    public Pose2d startingPose;

    public ThreeBallModeC() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.THREE_BALL_C,
                TrajectorySet.THREE_BALL_C_HEADINGS
            );
        startingPose = trajectory.getTrajectory().getInitialPose();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Three Ball C Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new RampUpShooterAction(Shooter.MID_VELOCITY), // make actual shooting vel
                trajectory,
                new ParallelAction(
                    new SeriesAction(
                        new ParallelAction(
                            new WaitUntilInsideRegion(
                                new Translation2d(149, 77), // make actual region to change hood
                                new Translation2d(180, 57)
                            ),
                            new TurretAction(110.2) // to be changed
                        ),
                        new ShootAction(true, true),
                        new ParallelAction(
                            new WaitUntilInsideRegion(
                                new Translation2d(111, 179), // make actual region to change hood
                                new Translation2d(146, 149)
                            ),
                            new TurretAction(180) // to be changed
                        ),
                        new ShootAction(true, true),
                        new WaitAction(2),
                        new ParallelAction( // stop all at end - make a stop action in the future
                            new CollectAction(false),
                            new RampUpShooterAction(Shooter.COAST_VELOCIY),
                            new ShootAction(false, false)
                        )
                    )
                )
            )
        );
    }
}
