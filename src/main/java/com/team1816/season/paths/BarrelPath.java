package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class BarrelPath implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
//            new Pose2d(46, 98, 0),
//            new Pose2d(168, 115, 0),
//            new Pose2d(158, 8, 180),
//            new Pose2d(130, 110, 10),
//            new Pose2d(278, 87, 30),
//            new Pose2d(253, 170, 176),
//            new Pose2d(190, 86, -60),
//            new Pose2d(271, 27, -25),
//            new Pose2d(342, 101, 132),
//            new Pose2d(207, 88, 180),
//            new Pose2d(143, 108, 180),
//            new Pose2d(32, 113, -180)
        );
        /*return List.of(
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90),
            new Pose2d(50, -50, 0),
            new Pose2d(100, 0, 90),
            new Pose2d(50, 50, 180),
            new Pose2d(0, 0, -90)
        );*/
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
