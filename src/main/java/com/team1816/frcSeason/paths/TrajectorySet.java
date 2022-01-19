package com.team1816.frcSeason.paths;

import com.team1816.frcSeason.paths.paths2020.*;
import com.team1816.frcSeason.paths.paths2021.*;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import javax.inject.Singleton;

@Singleton
public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    // 2020
    public static Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT;
    public static Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN;
    public static Trajectory<TimedState<Pose2dWithCurvature>> TUNE_DRIVETRAIN_REVERSE;
    public static Trajectory<TimedState<Pose2dWithCurvature>> LIVING_ROOM;
    public static Trajectory<TimedState<Pose2dWithCurvature>> SIX_BALL_ALLIANCE;
    public static Trajectory<TimedState<Pose2dWithCurvature>> AUTO_TRENCH_TURN_RIGHT;
    public static Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH;
    public static Trajectory<TimedState<Pose2dWithCurvature>> DRIVE_STRAIGHT_TRENCH_REVERSE;

    public static Trajectory<TimedState<Pose2dWithCurvature>> FEEDER_TO_TRENCH;
    public static Trajectory<TimedState<Pose2dWithCurvature>> TRENCH_TO_FEEDER;
    public static Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEA;
    public static Trajectory<TimedState<Pose2dWithCurvature>> FIVE_BALL_AUTO_OPPOSEB;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEA;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEB;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTA;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCEC;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_ALLIANCE_ALTB;

    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEA;
    public static Trajectory<TimedState<Pose2dWithCurvature>> EIGHT_BALL_AUTO_OPPOSEB;
    public static Trajectory<TimedState<Pose2dWithCurvature>> TEN_BALL_AUTO;

    public static Trajectory<TimedState<Pose2dWithCurvature>> BARREL;

    public TrajectorySet() {
        // 2020
        this.DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH = new DriveStraight(178, 100).generateTrajectory();
        this.DRIVE_STRAIGHT_TRENCH_REVERSE =
            new DriveStraight(100).generateReversedTrajectory();

        this.TUNE_DRIVETRAIN = new DriveStraight(180, 40).generateTrajectory();
        this.TUNE_DRIVETRAIN_REVERSE = new DriveStraight(-480).generateTrajectory();
        this.LIVING_ROOM = new LivingRoomPath().generateTrajectory();

        this.FIVE_BALL_AUTO_OPPOSEA = new DriveStraight(96).generateTrajectory();

        this.EIGHT_BALL_AUTO_ALLIANCEA =
            new DriveStraight(110).generateReversedTrajectory();

        this.BARREL = new BarrelPath().generateTrajectory();
    }
}
