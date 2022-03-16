package com.team1816.season.paths;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

@Singleton
public class TrajectorySet {

    // 2020
    public static Trajectory DRIVE_STRAIGHT_MODIFIED;
    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;

    public static List<Rotation2d> DRIVE_STRAIGHT_HEADINGS;
    public static List<Rotation2d> LIVING_ROOM_HEADINGS;

    //2022
    public static Trajectory TWO_BALL_A;
    public static Trajectory TWO_BALL_B;
    public static Trajectory TWO_BALL_C;
    public static Trajectory FIVE_BALL_A;
    public static Trajectory FIVE_BALL_B;
    public static Trajectory FIVE_BALL_C;

    public static List<Rotation2d> TWO_BALL_A_HEADINGS;
    public static List<Rotation2d> TWO_BALL_B_HEADINGS;
    public static List<Rotation2d> TWO_BALL_C_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_A_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_B_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_C_HEADINGS;

    public TrajectorySet() {
        // Trajectories
        // DRIVE_STRAIGHT = THIS USES OPEN LOOP - NOT TRAJECTORY
//        DRIVE_STRAIGHT_MODIFIED = new DriveStraightModified().generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(120, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();

        TWO_BALL_A = new TwoBallAutoA().generateTrajectory();
        TWO_BALL_B = new TwoBallAutoB().generateTrajectory();
        TWO_BALL_C = new TwoBallAutoC().generateTrajectory();
        FIVE_BALL_A = new FiveBallAutoA().generateTrajectory();
        FIVE_BALL_B = new FiveBallAutoB().generateTrajectory();
        FIVE_BALL_C = new FiveBallAutoC().generateTrajectory();

        // Heading lists

        DRIVE_STRAIGHT_HEADINGS = new DriveStraightModified().generateHeadings();
        LIVING_ROOM_HEADINGS = new LivingRoomPath().generateHeadings();

        TWO_BALL_A_HEADINGS = new TwoBallAutoA().generateHeadings();
        TWO_BALL_B_HEADINGS = new TwoBallAutoB().generateHeadings();
        TWO_BALL_C_HEADINGS = new TwoBallAutoC().generateHeadings();
        FIVE_BALL_A_HEADINGS = new FiveBallAutoA().generateHeadings();
        FIVE_BALL_B_HEADINGS = new FiveBallAutoB().generateHeadings();
        FIVE_BALL_C_HEADINGS = new FiveBallAutoC().generateHeadings();
    }
}
