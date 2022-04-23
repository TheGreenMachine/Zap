package com.team1816.season.auto.paths;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

@Singleton
public class TrajectorySet {

    // Testing and Tuning
    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;
    public static List<Rotation2d> LIVING_ROOM_HEADINGS;

    //2022
    public static Trajectory TWO_BALL_A;

    public static Trajectory TWO_BALL_B;
    public static Trajectory FOUR_BALL_B2;
    public static Trajectory FOUR_BALL_B3;

    public static Trajectory TWO_BALL_C;
    public static Trajectory FOUR_BALL_C2;
    public static Trajectory FOUR_BALL_C3;

    public static Trajectory FIVE_BALL_A;
    public static Trajectory FIVE_BALL_B;
    public static Trajectory FIVE_BALL_C;
    public static Trajectory FIVE_BALL_D;

    public static Trajectory ONE_BALL_A_B;
    public static Trajectory ONE_BALL_C_BORDER;

    public static List<Rotation2d> TWO_BALL_A_HEADINGS;

    public static List<Rotation2d> TWO_BALL_B_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_B2_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_B3_HEADINGS;

    public static List<Rotation2d> TWO_BALL_C_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_C2_HEADINGS;
    public static List<Rotation2d> FOUR_BALL_C3_HEADINGS;

    public static List<Rotation2d> FIVE_BALL_A_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_B_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_C_HEADINGS;
    public static List<Rotation2d> FIVE_BALL_D_HEADINGS;

    public static List<Rotation2d> ONE_BALL_A_B_HEADINGS;
    public static List<Rotation2d> ONE_BALL_C_BORDER_HEADINGS;

    public TrajectorySet() {
        // Trajectories
        // DRIVE_STRAIGHT = THIS USES OPEN LOOP - NOT TRAJECTORY
        TUNE_DRIVETRAIN = new DriveStraight(120, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();

        TWO_BALL_A = new TwoBallAutoA().generateTrajectory();

        TWO_BALL_B = new TwoBallAutoB().generateTrajectory();
        FOUR_BALL_B2 = new FourBallAutoB2().generateTrajectory();
        FOUR_BALL_B3 = new FourBallAutoB3().generateTrajectory();

        TWO_BALL_C = new TwoBallAutoC().generateTrajectory();
        FOUR_BALL_C2 = new FourBallAutoC2().generateTrajectory();
        FOUR_BALL_C3 = new FourBallAutoC3().generateTrajectory();

        FIVE_BALL_A = new FiveBallAutoA().generateTrajectory();
        FIVE_BALL_B = new FiveBallAutoB().generateTrajectory();
        FIVE_BALL_C = new FiveBallAutoC().generateTrajectory();
        FIVE_BALL_D = new FiveBallAutoD().generateTrajectory();

        ONE_BALL_A_B = new OneBallA_B().generateTrajectory();
        ONE_BALL_C_BORDER = new OneBallAutoC_Border().generateTrajectory();

        // Heading lists
        LIVING_ROOM_HEADINGS = new LivingRoomPath().generateHeadings();

        TWO_BALL_A_HEADINGS = new TwoBallAutoA().generateHeadings();

        TWO_BALL_B_HEADINGS = new TwoBallAutoB().generateHeadings();
        FOUR_BALL_B2_HEADINGS = new FourBallAutoB2().generateHeadings();
        FOUR_BALL_B3_HEADINGS = new FourBallAutoB3().generateHeadings();

        TWO_BALL_C_HEADINGS = new TwoBallAutoC().generateHeadings();
        FOUR_BALL_C2_HEADINGS = new FourBallAutoC2().generateHeadings();
        FOUR_BALL_C3_HEADINGS = new FourBallAutoC3().generateHeadings();

        FIVE_BALL_A_HEADINGS = new FiveBallAutoA().generateHeadings();
        FIVE_BALL_B_HEADINGS = new FiveBallAutoB().generateHeadings();
        FIVE_BALL_C_HEADINGS = new FiveBallAutoC().generateHeadings();
        FIVE_BALL_D_HEADINGS = new FiveBallAutoD().generateHeadings();

        ONE_BALL_A_B_HEADINGS = new OneBallA_B().generateHeadings();
        ONE_BALL_C_BORDER_HEADINGS = new OneBallAutoC_Border().generateHeadings();
    }
}
