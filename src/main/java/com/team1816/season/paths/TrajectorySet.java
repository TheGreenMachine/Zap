package com.team1816.season.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import com.google.inject.Singleton;
import java.util.List;

@Singleton
public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    // 2020
    public static Trajectory DRIVE_STRAIGHT;
    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;
    public static Trajectory BARREL;

    public static List<Rotation2d> LIVING_ROOM_HEADINGS;
    public static List<Rotation2d> BARREL_HEADINGS;

    public TrajectorySet() {
        // Trajectories
        DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(180, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        BARREL = new BarrelPath().generateTrajectory();

        // Heading lists
        LIVING_ROOM_HEADINGS = new LivingRoomPath().generateHeadings();
        BARREL_HEADINGS = new BarrelPath().generateHeadings();
    }
}
