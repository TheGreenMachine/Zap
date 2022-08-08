package com.team1816.lib.auto.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public abstract class AutoPath {

    Trajectory trajectory;
    List<Rotation2d> headings;

    protected abstract List<Pose2d> getWaypoints();

    protected abstract List<Rotation2d> getWaypointHeadings();

    protected abstract boolean usingApp(); // just to check whether to add the starting pose of .5 X and 3.5 Y

    public Trajectory getAsTrajectory() {
        if (trajectory == null) {
            trajectory = PathUtil.generateTrajectory(usingApp(), getWaypoints());
        }
        return trajectory;
    }

    public List<Rotation2d> getAsTrajectoryHeadings() {
        if (headings == null) {
            headings =
                PathUtil.generateHeadings(
                    usingApp(),
                    getWaypoints(),
                    getWaypointHeadings()
                );
        }
        return headings;
    }
}
