package com.team1816.lib.paths;

import com.team1816.lib.hardware.RobotFactory;
import com.team1816.season.Constants;
import com.team254.lib.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;
import jdk.jshell.spi.ExecutionControl;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    // velocities are in/sec
    double kMaxVelocity = Units.inches_to_meters(
        RobotFactory.getInstance().getConstant("maxVel")
    );
    double kMaxAccel = Units.inches_to_meters(
        RobotFactory.getInstance().getConstant("maxAccel")
    );

    List<Pose2d> buildWaypoints();

    default Trajectory generateTrajectory() {
        return generateBaseTrajectory(isReversed(), buildWaypoints());
    }

    default Trajectory generateReversedTrajectory()
        throws ExecutionControl.NotImplementedException {
        throw new ExecutionControl.NotImplementedException("TODO");
    }

    private Trajectory generateBaseTrajectory(
        boolean isReversed,
        List<Pose2d> waypoints
    ) {
        List<Pose2d> waypointsMeters = new ArrayList<>();
        for(Pose2d pose2d: waypoints){
            waypointsMeters.add(new Pose2d(Units.inches_to_meters(pose2d.getX()), Units.inches_to_meters(pose2d.getY()), pose2d.getRotation()));
        }
        TrajectoryConfig config = new TrajectoryConfig(kMaxVelocity, kMaxAccel);
        var baseTrajectory = TrajectoryGenerator.generateTrajectory(waypointsMeters, config); // we want to use the config here not the base trajecory!
        return baseTrajectory.transformBy(
            new Transform2d(
                Constants.StartingPose.getTranslation(),
                Constants.StartingPose.getRotation()
            )
        );
    }

    boolean isReversed();
}
