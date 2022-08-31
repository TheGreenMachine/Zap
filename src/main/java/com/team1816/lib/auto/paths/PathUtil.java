package com.team1816.lib.auto.paths;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxAccelMeters;
import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class PathUtil {

    private static final double kMaxVelocity = kPathFollowingMaxVelMeters;
    private static final double kMaxAccel = kPathFollowingMaxAccelMeters;

    public static Trajectory generateTrajectory(
        boolean usingApp,
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
        var baseTrajectory = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            waypointsMeters,
            config
        );
        if (!usingApp) {
            baseTrajectory =
                baseTrajectory.transformBy(
                    new Transform2d(
                        Constants.kDefaultZeroingPose.getTranslation(),
                        Constants.kDefaultZeroingPose.getRotation()
                    )
                );
        }
        return baseTrajectory;
    }

    public static List<Rotation2d> generateHeadings(
        boolean usingApp,
        List<Pose2d> waypoints,
        List<Rotation2d> swerveHeadings
    ) {
        if (waypoints == null || swerveHeadings == null) {
            return null;
        }
        double startX = .5;
        double startY = Constants.fieldCenterY;

        Trajectory trajectory = generateTrajectory(usingApp, waypoints);
        List<Pose2d> waypointsMeters = new ArrayList<>();
        if (usingApp) {
            startX = 0;
            startY = 0;
        }
        for (Pose2d pose2d : waypoints) {
            waypointsMeters.add(
                new Pose2d(
                    Units.inchesToMeters(pose2d.getX()) + startX,
                    Units.inchesToMeters(pose2d.getY()) + startY,
                    pose2d.getRotation()
                )
            );
        }

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
                    swerveHeadings.get(nextCheckpoint).getDegrees() -
                    swerveHeadings.get(nextCheckpoint - 1).getDegrees()
                );
            double timeBetweenWaypoints =
                waypointTimes.get(nextCheckpoint) - waypointTimes.get(nextCheckpoint - 1);
            double dHeading = totalDHeading / timeBetweenWaypoints; // change in heading per state

            for (int i = iStart; i < iEnd; i++) {
                generatedHeadings.add(
                    Rotation2d.fromDegrees(
                        swerveHeadings.get(nextCheckpoint - 1).getDegrees() + // last waypoint's heading in degrees
                        dHeading *
                        (
                            trajectory.getStates().get(i).timeSeconds -
                            waypointTimes.get(nextCheckpoint - 1)
                        ) // change in heading * current time between waypoints
                    )
                );
            }
        }
        // add the end rotation to make sure that last rotation imput is the last heading - band-aid fix.
        generatedHeadings.add(swerveHeadings.get(swerveHeadings.size() - 1));

        return generatedHeadings;
    }
}
