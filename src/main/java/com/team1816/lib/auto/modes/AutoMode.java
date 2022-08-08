package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoMode {

    private static final long looperDtInMS = (long) (Constants.kLooperDt * 1000);

    private boolean needsStop;

    protected List<TrajectoryAction> trajectoryActions;
    protected Pose2d initialPose;

    protected AutoMode() {}

    protected AutoMode(List<TrajectoryAction> trajectoryActions) {
        this.trajectoryActions = trajectoryActions;
        initialPose = trajectoryActions.get(0).getTrajectory().getInitialPose();
    }

    public void run() {
        start();

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE STOPPED EARLY ! ! !", false);
        }

        done();
    }

    private void start() {
        System.out.println("Starting " + this.getClass().getName());
        needsStop = false;
    }

    // what actions each auto mode runs when thread calls autoMode's run method
    protected abstract void routine() throws AutoModeEndedException;

    private void done() {
        System.out.println(this.getClass().getName() + " Done");
    }

    public void stop() {
        needsStop = true;
    }

    protected void runAction(Action action) throws AutoModeEndedException {
        action.start();

        // Run action, stop action on interrupt or done
        while (!action.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            action.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public Trajectory getCurrentTrajectory() {
        if (trajectoryActions != null && trajectoryActions.size() > 0) {
            for (int i = 0; i < trajectoryActions.size(); i++) {
                if (!trajectoryActions.get(i).isFinished()) {
                    return trajectoryActions.get(i).getTrajectory();
                }
            }
        }
        return new Trajectory();
    }

    public Pose2d getInitialPose() {
        if (initialPose == null) {
            return Constants.kDefaultZeroingPose;
        }
        return initialPose;
    }
}
