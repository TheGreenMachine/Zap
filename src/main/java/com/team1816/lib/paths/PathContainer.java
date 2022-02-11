package com.team1816.lib.paths;

import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.math.Conversions;
import com.team1816.season.Constants;
import com.team254.lib.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    List<Rotation2d> buildHeadings();

    default Trajectory generateTrajectory() {
        return generateBaseTrajectory(isReversed(), buildWaypoints());
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
        var baseTrajectory = TrajectoryGenerator.generateTrajectory(waypointsMeters, config);
        return baseTrajectory.transformBy(
            new Transform2d(
                Constants.StartingPose.getTranslation(),
                Constants.StartingPose.getRotation()
            )
        );
    }

    boolean isReversed();

    default List<Rotation2d> generateHeadings(){
        Trajectory trajectory = generateTrajectory();
        List<Pose2d> waypointsMeters = new ArrayList<>();
        for(Pose2d pose2d: buildWaypoints()){
            waypointsMeters.add(new Pose2d(Units.inches_to_meters(pose2d.getX()) + .5, Units.inches_to_meters(pose2d.getY()) + 3.5, pose2d.getRotation()));
        }
        List<Rotation2d> waypointHeadings = buildHeadings();

        //get times and indices
        List<Double> waypointTimes = new ArrayList<>();
        List<Integer> waypointIndexes = new ArrayList<>();
        int iWaypointCheckpoint = 0;
        for(Pose2d pose2d : waypointsMeters){
            for(int i = iWaypointCheckpoint; i < trajectory.getStates().size(); i++){
                var point = trajectory.getStates().get(i).poseMeters;
                if(point.equals(pose2d)){ // Conversions.epsilonEquals(point, pose2d, .0001
                    waypointTimes.add(trajectory.getStates().get(i).timeSeconds);
                    waypointIndexes.add(i);
                    break;
                }
                iWaypointCheckpoint++;
            }
        }

        // generate list of rotation2Ds equivalent to trajectory length - for each state a heading
        List<Rotation2d> generatedHeadings = new ArrayList<>();
        for(int nextCheckpoint = 1; nextCheckpoint < waypointsMeters.size(); nextCheckpoint++){
            int iStart = waypointIndexes.get(nextCheckpoint - 1);
            int iEnd = waypointIndexes.get(nextCheckpoint);
            double totalDHeading = ( // total change in heading between two points in degrees
                waypointHeadings.get(nextCheckpoint).getDegrees() - waypointHeadings.get(nextCheckpoint - 1).getDegrees()
            );
            double timeBetweenWaypoints = waypointTimes.get(nextCheckpoint) - waypointTimes.get(nextCheckpoint - 1);
            double dHeading = totalDHeading / timeBetweenWaypoints; // change in heading per state

            for(int i = iStart; i < iEnd; i++){
                generatedHeadings.add(
                    Rotation2d.fromDegrees(
                        waypointHeadings.get(nextCheckpoint - 1).getDegrees() + // last waypoint's heading in degrees
                            dHeading * (trajectory.getStates().get(i).timeSeconds - waypointTimes.get(nextCheckpoint - 1)) // change in heading * current time between waypoints
                    )
                );
                //System.out.println(generatedHeadings.get(i).getDegrees() + " = generated headings");
            }

        }
        // add the end rotation to make sure that last rotation imput is the last heading - kind of a band-aid fix.
        generatedHeadings.add(waypointHeadings.get(waypointHeadings.size() - 1));

        return  generatedHeadings;
    }

}
