package com.team1816.lib.paths;

import com.team1816.frcSeason.Robot;
import com.team1816.frcSeason.planners.SwerveMotionPlanner;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team1816.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    // velocities are in/sec
    double kMaxVelocity = Robot.getFactory().getConstant("maxVel");
    double kMaxAccel = Robot.getFactory().getConstant("maxAccel");
    double kMaxCentripetalAccel = Robot
        .getFactory()
        .getConstant("maxCentripetalAccel", 100.0);
    double kMaxDecel =
        (
            Robot.getFactory().getConstant("maxDecel", -1) == -1
                ? kMaxAccel
                : Robot.getFactory().getConstant("maxDecel")
        );
    double kMaxVoltage = 9.0;

    Path buildPath();

    List<Pose2d> buildWaypoints();

    default Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory() {
        return generateBaseTrajectory(isReversed(), buildWaypoints());
    }

    default Trajectory<TimedState<Pose2dWithCurvature>> generateReversedTrajectory() {
        return TrajectoryUtil.mirrorTimed(
            generateBaseTrajectory(!isReversed(), reverseWaypoints(buildWaypoints()))
        );
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> generateBaseTrajectory(
        boolean isReversed,
        List<Pose2d> waypoints
    ) {
        return (new SwerveMotionPlanner()).generateTrajectory(
                isReversed,
                waypoints,
                Arrays.asList(
                    new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
                ),
                getMaxVelocity(),
                kMaxAccel,
                kMaxDecel,
                kMaxVoltage
            );
    }

    private List<Pose2d> reverseWaypoints(List<Pose2d> waypoints) {
        return waypoints
            .stream()
            .map(
                pose ->
                    new Pose2d(
                        -pose.getTranslation().x(),
                        pose.getTranslation().y(),
                        pose.getRotation()
                    )
            )
            .collect(Collectors.toList());
    }

    boolean isReversed();

    default double getMaxVelocity() {
        return kMaxVelocity;
    }

    default Rotation2d getTargetHeading() {
        return Rotation2d.identity();
    }
}
