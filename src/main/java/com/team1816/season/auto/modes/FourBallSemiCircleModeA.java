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

public class FourBallSemiCircleModeA extends AutoModeBase {

    public Pose2d startingPose;
    private TrajectoryAction trajectory1;

    public FourBallSemiCircleModeA() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_A,
                TrajectorySet.FIVE_BALL_A_HEADINGS
            );
        startingPose = trajectory.getTrajectory().getInitialPose();
        trajectory1 =
            new TrajectoryAction(
                TrajectorySet.FOUR_BALL_SEMICIRCLE_A,
                TrajectorySet.FOUR_BALL_SEMICIRCLE_A_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Four Ball Semicircle A Mode");
        runAction(new WaitAction(.5));
        //still needs to be changed
        runAction(
            new SeriesAction(
                new CollectAction(true),
                new RampUpShooterAction(Shooter.MID_VELOCITY), // make actual shooting vel
                new TurretAction(Turret.CARDINAL_WEST), // setting this doesn't seem to work right in simulator - magically relative to field and not the robot
                trajectory,
                new ParallelAction(
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(210, 180)
                        ),
                        new ShootAction(true, true),
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(205, 130)
                        ),
                        new ShootAction(true, false),
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(220, 80)
                        ),
                        new ShootAction(true, true)
                    ),
                    trajectory1
                ),
                new RampUpShooterAction(Shooter.FAR_VELOCITY),
                new WaitAction(2),
                new ParallelAction( // stop all at end - make a stop action in the future
                    new CollectAction(false),
                    new RampUpShooterAction(Shooter.COAST_VELOCITY),
                    new ShootAction(false, false)
                )
            )
        );
    }
}
