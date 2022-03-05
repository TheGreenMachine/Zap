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

    public static List<Rotation2d> DRIVE_STRAIGHT_HEADINGS;
    public static List<Rotation2d> LIVING_ROOM_HEADINGS;

    //2022
    public static Trajectory ONE_BALL_A;
    public static Trajectory ONE_BALL_B;
    public static Trajectory TWO_BALL_A;
    public static Trajectory TWO_BALL_B;
    public static Trajectory TWO_BALL_C;
    public static Trajectory THREE_BALL_A;
    public static Trajectory THREE_BALL_B;
    public static Trajectory THREE_BALL_C;
    public static Trajectory FOUR_BALL_SEMICIRCLE_A;
    public static Trajectory FOUR_BALL_SEMICIRCLE_B;
    public static Trajectory FOUR_BALL_C;
    public static Trajectory FIVE_BALL_A;
    public static Trajectory FIVE_BALL_B;
    public static Trajectory FIVE_BALL_C;
    public static Trajectory FIVE_BALL_D; // different

    public static List<Rotation2d> ONE_BALL_A_HEADINGS;
    public static List<Rotation2d> ONE_BALL_B_HEADINGS;
    public static List<Rotation2d> TWO_BALL_A_HEADINGS;
    public static List<Rotation2d> TWO_BALL_B_HEADINGS;
    public static List<Rotation2d> TWO_BALL_C_HEADINGS;
    public static List<Rotation2d> THREE_BALL_A_HEADINGS;
    public static List<Rotation2d> THREE_BALL_B_HEADINGS;
    public static List<Rotation2d> THREE_BALL_C_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_SEMICIRCLE_A_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_SEMICIRCLE_B_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_C_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_A_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_B_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_C_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_D_HEADINGS;

    public TrajectorySet() {
        // Trajectories
        //DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        DRIVE_STRAIGHT = new DriveStraightModified().generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(264, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        ONE_BALL_A = new OneBallAutoA().generateTrajectory();
        ONE_BALL_B = new OneBallAutoB().generateTrajectory();
        TWO_BALL_A = new TwoBallAutoA().generateTrajectory();
        TWO_BALL_B = new TwoBallAutoB().generateTrajectory();
        TWO_BALL_C = new TwoBallAutoC().generateTrajectory();
        FOUR_BALL_SEMICIRCLE_A = new FourBallSemiCircleAutoA().generateTrajectory();
        FOUR_BALL_SEMICIRCLE_B = new FourBallSemiCircleAutoB().generateTrajectory();
        FOUR_BALL_C = new FourBallAutoC().generateTrajectory();
        FIVE_BALL_A = new FiveBallAutoA().generateTrajectory();
        FIVE_BALL_B = new FiveBallAutoB().generateTrajectory();
        FIVE_BALL_C = new FiveBallAutoC().generateTrajectory();
        FIVE_BALL_D = new FiveBallAutoE().generateTrajectory();

        // Heading lists

        DRIVE_STRAIGHT_HEADINGS = new DriveStraightModified().generateHeadings();
        LIVING_ROOM_HEADINGS = new LivingRoomPath().generateHeadings();
        ONE_BALL_A_HEADINGS = new OneBallAutoA().generateHeadings();
        ONE_BALL_B_HEADINGS = new OneBallAutoB().generateHeadings();
        TWO_BALL_A_HEADINGS = new TwoBallAutoA().generateHeadings();
        TWO_BALL_B_HEADINGS = new TwoBallAutoB().generateHeadings();
        TWO_BALL_C_HEADINGS = new TwoBallAutoC().generateHeadings();
        FOUR_BALL_SEMICIRCLE_A_HEADINGS = new FourBallSemiCircleAutoA().generateHeadings();
        FOUR_BALL_SEMICIRCLE_B_HEADINGS = new FourBallSemiCircleAutoB().generateHeadings();
        FOUR_BALL_C_HEADINGS = new FourBallAutoC().generateHeadings();
        FIVE_BALL_A_HEADINGS = new FiveBallAutoA().generateHeadings();
        FIVE_BALL_B_HEADINGS = new FiveBallAutoB().generateHeadings();
        FIVE_BALL_C_HEADINGS = new FiveBallAutoC().generateHeadings();
        FIVE_BALL_D_HEADINGS = new FiveBallAutoE().generateHeadings();
    }
}
