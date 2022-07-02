package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoMode {

    private static final long looperDtInMS = (long) (Constants.kLooperDt * 1000);
    private Thread autoModeThread = new Thread(this::run);
    protected List<TrajectoryAction> trajectoryActions;
    protected Pose2d initialPose;

    protected AutoMode() {}

    protected AutoMode(List<TrajectoryAction> trajectoryActions) {
        this.trajectoryActions = trajectoryActions;
        initialPose = trajectoryActions.get(0).getTrajectory().getInitialPose();
    }

    protected abstract void routine();

    public void start() {
        if (autoModeThread != null) {
            autoModeThread.start();
        }
    }

    public void run() {
        try {
            if (!autoModeThread.isAlive()) {
                throw new AutoModeEndedException();
            }

            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE ENDED EARLY!", false);
            return;
        }

        done();
    }

    public void done() {
        System.out.println("Auto mode done");
        reset();
    }

    public void reset() {
        autoModeThread.stop();
        autoModeThread = new Thread(this::run);
    }

    public void runAction(Action action) {
        action.start();

        // Run action, stop action on interrupt or done
        while (!action.isFinished()) {
            action.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public Pose2d getInitialPose() {
        if (initialPose == null) {
            return Constants.StartingPose;
        }
        return initialPose;
    }
}
