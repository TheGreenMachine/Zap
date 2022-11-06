package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.*;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class FiveBallMode extends AutoMode {

    public FiveBallMode() {
        super(
            List.of(
                new TrajectoryAction(new FiveBallPathA()),
                new TrajectoryAction(new FiveBallPathB()),
                new TrajectoryAction(new FiveBallPathC()),
                new TrajectoryAction(new FiveBallPathD())
            )
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
                    //new TurretAction(Turret.kWest + 15), // setting this doesn't seem to work right in simulator - magically relative to field and not the robot
                    new AbsoluteTurretAction(),
                    trajectoryActions.get(0)
                ),
                new ParallelAction(
                    trajectoryActions.get(1),
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
                        //new TurretAction(Turret.kWest - 10),
                        new AbsoluteTurretAction()
                    )
                ),
                new WaitAction(1),
                new ShootAction(false, true),
                trajectoryActions.get(2),
                new WaitAction(1),
                new ParallelAction(
                    new RampUpShooterAction(14000),
                    //new TurretAction(Turret.kSouth),
                    new AbsoluteTurretAction(),
                    trajectoryActions.get(3)
                ),
                new ShootAction(true, true),
                new WaitAction(2),
                new StopAction(false)
            )
        );
    }
}
