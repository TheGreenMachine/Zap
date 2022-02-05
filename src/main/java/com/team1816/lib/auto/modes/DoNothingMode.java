package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import edu.wpi.first.math.trajectory.Trajectory;

public class DoNothingMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }

    @Override
    public Trajectory getTrajectory() {
        return null;
    }
}
