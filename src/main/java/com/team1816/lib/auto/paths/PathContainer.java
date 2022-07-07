package com.team1816.lib.auto.paths;

import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    // velocities are in/sec
    double kMaxVelocity = Constants.kPathFollowingMaxVelMeters;
    double kMaxAccel = Constants.kPathFollowingMaxAccelMeters;

    List<Pose2d> buildWaypoints();
    List<Rotation2d> buildHeadings();

    default Trajectory generateTrajectory() {
        return generateBaseTrajectory(usingApp(), buildWaypoints());
    }

    private Trajectory generateBaseTrajectory(
        boolean isReversed,
        List<Pose2d> waypoints
    ) {
        List<Pose2d> waypointsMeters = new ArrayList<>();
        for (Pose2d pose2d : waypoints) {
            waypointsMeters.add(
                new Pose2d(
                    // manually adding the starting pose to each waypoint - wpilib transformBy acts up when transforming FiveBall - bug?
                    Units.inchesToMeters(pose2d.getX()), // + Constants.StartingPose.getTranslation().getX(),
                    Units.inchesToMeters(pose2d.getY()), // + Constants.StartingPose.getTranslation().getY(),
                    pose2d.getRotation() // .plus(Constants.StartingPose.getRotation())
                )
            );
        }
        TrajectoryConfig config = new TrajectoryConfig(kMaxVelocity, kMaxAccel);
        var baseTrajectory = TrajectoryGenerator.generateTrajectory(
            waypointsMeters,
            config
        );
        if (!isReversed) {
            baseTrajectory =
                baseTrajectory.transformBy(
                    new Transform2d(
                        Constants.StartingPose.getTranslation(),
                        Constants.StartingPose.getRotation()
                    )
                );
        }
        return baseTrajectory;
    }

    boolean usingApp(); // just to check whether to add the starting pose of .5 X and 3.5 Y

    default List<Rotation2d> generateHeadings() {
        double startX = .5;
        double startY = Constants.fieldCenterY;

        Trajectory trajectory = generateTrajectory();
        List<Pose2d> waypointsMeters = new ArrayList<>();
        if (usingApp()) {
            startX = 0;
            startY = 0;
        }
        for (Pose2d pose2d : buildWaypoints()) {
            waypointsMeters.add(
                new Pose2d(
                    Units.inchesToMeters(pose2d.getX()) + startX,
                    Units.inchesToMeters(pose2d.getY()) + startY,
                    pose2d.getRotation()
                )
            );
        }
        List<Rotation2d> waypointHeadings = buildHeadings();

        //get times and indices
        List<Double> waypointTimes = new ArrayList<>();
        List<Integer> waypointIndexes = new ArrayList<>();
        int iWaypointCheckpoint = 0;
        for (Pose2d waypointPose2d : waypointsMeters) {
            for (int i = iWaypointCheckpoint; i < trajectory.getStates().size(); i++) {
                var trajectoryPose2d = trajectory.getStates().get(i).poseMeters;
                if (trajectoryPose2d.equals(waypointPose2d)) { // Conversions.epsilonEquals(point, pose2d, .0001
                    waypointTimes.add(trajectory.getStates().get(i).timeSeconds);
                    waypointIndexes.add(i);
                    break;
                }
                iWaypointCheckpoint++;
            }
        }

        // generate list of rotation2Ds equivalent to trajectory length - for each state a heading
        List<Rotation2d> generatedHeadings = new ArrayList<>();
        for (
            int nextCheckpoint = 1;
            nextCheckpoint < waypointsMeters.size();
            nextCheckpoint++
        ) {
            int iStart = waypointIndexes.get(nextCheckpoint - 1);
            int iEnd = waypointIndexes.get(nextCheckpoint);
            double totalDHeading =
                ( // total change in heading between two points in degrees
                    waypointHeadings.get(nextCheckpoint).getDegrees() -
                    waypointHeadings.get(nextCheckpoint - 1).getDegrees()
                );
            double timeBetweenWaypoints =
                waypointTimes.get(nextCheckpoint) - waypointTimes.get(nextCheckpoint - 1);
            double dHeading = totalDHeading / timeBetweenWaypoints; // change in heading per state

            for (int i = iStart; i < iEnd; i++) {
                generatedHeadings.add(
                    Rotation2d.fromDegrees(
                        waypointHeadings.get(nextCheckpoint - 1).getDegrees() + // last waypoint's heading in degrees
                        dHeading *
                        (
                            trajectory.getStates().get(i).timeSeconds -
                            waypointTimes.get(nextCheckpoint - 1)
                        ) // change in heading * current time between waypoints
                    )
                );
                //System.out.println(generatedHeadings.get(i).getDegrees() + " = generated headings");
            }
        }
        // add the end rotation to make sure that last rotation imput is the last heading - band-aid fix.
        generatedHeadings.add(waypointHeadings.get(waypointHeadings.size() - 1));

        return generatedHeadings;
    }
}
