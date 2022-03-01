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

public class ThreeBallModeB extends AutoModeBase {
    public Pose2d startingPose;

    public ThreeBallModeB() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.THREE_BALL_B,
                TrajectorySet.THREE_BALL_B_HEADINGS
            );
        startingPose = trajectory.getTrajectory().getInitialPose();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Three Ball B Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new RampUpShooterAction(Shooter.MID_VELOCITY), // make actual shooting vel
                trajectory,
                new ParallelAction(
                    new SeriesAction(
                        // STUFF BELOW needs to be changed
                        new ParallelAction(
                            new WaitUntilInsideRegion(
                                new Translation2d(198, 23), // make actual region to change hood
                                new Translation2d(234, 57)
                            ),
                            new TurretAction(228.89) // to be changed
                        ),
                        new ShootAction(true, true),
                        new ParallelAction(
                            new WaitUntilInsideRegion(
                                new Translation2d(285, 5), // make actual region to change hood
                                new Translation2d(306, 21)
                            ),
                            new TurretAction(264.23) // to be changed
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
