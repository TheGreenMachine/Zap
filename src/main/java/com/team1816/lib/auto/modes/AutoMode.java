package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.season.Constants;
import com.team1816.season.auto.paths.DriveStraight;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoMode {

    private static final long looperDtInMS = (long) Constants.kLooperDt * 1000;
    protected boolean isActive = false;
    protected TrajectoryAction trajectoryAction;

    protected abstract void routine() throws AutoModeEndedException;

    public Trajectory getTrajectory() {
        if (trajectoryAction == null) {
            trajectoryAction = new TrajectoryAction(new DriveStraight());
        }
        return trajectoryAction.getTrajectory();
    }

    public void run() {
        isActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        isActive = false;
    }

    public boolean isActive() {
        return isActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive) {
            throw new AutoModeEndedException();
        }

        return true;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }
}
