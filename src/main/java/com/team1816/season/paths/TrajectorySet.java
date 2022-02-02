package com.team1816.season.paths;

import com.team1816.season.paths.paths2020.*;
import com.team1816.season.paths.paths2021.*;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.math.trajectory.Trajectory;

import javax.inject.Singleton;

@Singleton
public class TrajectorySet {

    private static TrajectorySet INSTANCE;

    // 2020
    public static Trajectory DRIVE_STRAIGHT;
    public static Trajectory TUNE_DRIVETRAIN;
    public static Trajectory LIVING_ROOM;
    public static Trajectory BARREL;

    public TrajectorySet() {
        // 2020
        DRIVE_STRAIGHT = new DriveStraight(36).generateTrajectory();
        TUNE_DRIVETRAIN = new DriveStraight(180, 40).generateTrajectory();
        LIVING_ROOM = new LivingRoomPath().generateTrajectory();
        BARREL = new BarrelPath().generateTrajectory();
    }
}
