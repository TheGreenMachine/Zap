package com.team1816.season.paths;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

@Singleton
public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    // 2020
    public static Trajectory DRIVE_STRAIGHT;
    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;
    public static Trajectory FIVE_BALL_A;
    public static Trajectory FIVE_BALL_B;
    public static Trajectory FIVE_BALL_C;
    public static Trajectory FIVE_BALL_D;



    public static List<Rotation2d> LIVING_ROOM_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_A_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_B_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_C_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_D_HEADINGS;



    public TrajectorySet() {
        // Trajectories
        DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(180, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        FIVE_BALL_A = new FiveBallAutoA().generateTrajectory();
        FIVE_BALL_B = new FiveBallAutoB().generateTrajectory();
        FIVE_BALL_C = new FiveBallAutoC().generateTrajectory();
        FIVE_BALL_D = new FiveBallAutoD().generateTrajectory();


        // Heading lists
        LIVING_ROOM_HEADINGS = new LivingRoomPath().generateHeadings();
        FIVE_BALL_A_HEADINGS = new FiveBallAutoA().generateHeadings();
        FIVE_BALL_B_HEADINGS = new FiveBallAutoB().generateHeadings();
        FIVE_BALL_C_HEADINGS = new FiveBallAutoC().generateHeadings();
        FIVE_BALL_D_HEADINGS = new FiveBallAutoD().generateHeadings();

    }
}
