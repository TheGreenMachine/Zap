package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.*;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;
import edu.wpi.first.math.geometry.Translation2d;

public class FiveBallMode extends AutoModeBase {

    public FiveBallMode() {
        trajectory =
            new TrajectoryAction(
                TrajectorySet.FIVE_BALL_B,
                TrajectorySet.FIVE_BALL_B_HEADINGS
            );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Five Ball Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                // starting actions before paths run
                new CollectAction(true),
                new RampUpShooterAction(13000), // make actual shooting vel
                new TurretAction(Turret.CARDINAL_WEST + 15), // setting this doesn't seem to work right in simulator - magically relative to field and not the robot
                new ParallelAction(
                    // paths
                    new SeriesAction(
                        trajectory
                    ),
                    // actions to take during the path
                    new SeriesAction(
                        //                        new ShootAction(true, true),
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(210, 180),
                            "1st, ally ball"
                        ),
                        new ShootAction(true, true),
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(205, 130),
                            "2nd, enemy ball"
                        ),
                        new ShootAction(true, false),
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0), // make actual region to change hood
                            new Translation2d(220, 80),
                            "3rd, ally ball"
                        ),
                        new ShootAction(true, true),
//                        another wait until inside region action here
//                        new TurretAction(Turret.CARDINAL_NORTH),
                        new WaitAction(4),
                        new RampUpShooterAction(14000),
                        new TurretAction(Turret.CARDINAL_SOUTH), // tune these two
                        new ShootAction(true, true),
                        new WaitAction(2),
                        new StopAction(false)
                    )
                )
            )
        );
//        runAction(); // stop all at end
    }
}
