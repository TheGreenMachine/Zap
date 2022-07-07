package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;
import edu.wpi.first.math.geometry.Translation2d;

public class FiveBallMode extends AutoModeBase {

    private final TrajectoryAction trajectory1;
    private final TrajectoryAction trajectory2;
    private final TrajectoryAction trajectory3;

    public FiveBallMode() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_A,
                TrajectorySet.FIVE_BALL_A_HEADINGS
            );
        trajectory1 =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_B,
                TrajectorySet.FIVE_BALL_B_HEADINGS
            );
        trajectory2 =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_C,
                TrajectorySet.FIVE_BALL_C_HEADINGS
            );
        trajectory3 =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_D,
                TrajectorySet.FIVE_BALL_D_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Five Ball Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new ParallelAction(
                    new CollectAction(true),
                    new RampUpShooterAction(13000), // make actual shooting vel
                    new TurretAction(Turret.WEST + 15), // setting this doesn't seem to work right in simulator - magically relative to field and not the robot
                    trajectory
                ),
                new ParallelAction(
                    trajectory1,
                    new SeriesAction(
                        new ShootAction(true, true),
                        new WaitUntilInsideRegion(
                            new Translation2d(160, 99), // make actual region to change hood
                            new Translation2d(203, 134),
                            "2nd, enemy ball"
                        ),
                        new ShootAction(true, false),
                        new WaitUntilInsideRegion(
                            new Translation2d(184, 46), // make actual region to change hood
                            new Translation2d(241, 69),
                            "3rd, ally ball"
                        ),
                        new ShootAction(true, true),
                        new WaitUntilInsideRegion(
                            new Translation2d(265, 25),
                            new Translation2d(306, 47),
                            "4th, ally ball"
                        ),
                        new TurretAction(Turret.WEST - 10)
                    )
                ),
                new WaitAction(1),
                new ShootAction(false, true),
                trajectory2,
                new WaitAction(1),
                new ParallelAction(
                    new RampUpShooterAction(14000),
                    new TurretAction(Turret.SOUTH),
                    trajectory3
                ),
                new ShootAction(true, true),
                new WaitAction(2),
                new StopAction(false)
            )
        );
    }
}
