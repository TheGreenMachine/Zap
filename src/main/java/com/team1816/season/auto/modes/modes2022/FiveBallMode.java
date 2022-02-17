package com.team1816.season.auto.modes.modes2022;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.actions2020.TurretAction;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.Turret;

public class FiveBallMode extends AutoModeBase {

    private TrajectoryAction trajectory1;

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
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(new SeriesAction(trajectory, trajectory1));
        runAction(new TurretAction(Turret.CARDINAL_WEST));
    }
}
