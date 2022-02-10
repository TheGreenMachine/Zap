package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BarrelPath implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(46, 98, Rotation2d.fromDegrees(0)), //patch fix to resolve negative array size exception
            new Pose2d(168, 115, Rotation2d.fromDegrees(0)),
            new Pose2d(158, 8, Rotation2d.fromDegrees(180)),
            new Pose2d(130, 110, Rotation2d.fromDegrees(10)),
            new Pose2d(278, 87, Rotation2d.fromDegrees(30))
//            new Pose2d(253, 170, Rotation2d.fromDegrees(176)),
//            new Pose2d(190, 86, Rotation2d.fromDegrees(-60)),
//            new Pose2d(271, 27, Rotation2d.fromDegrees(-25)),
//            new Pose2d(342, 101, Rotation2d.fromDegrees(132)),
//            new Pose2d(207, 88, Rotation2d.fromDegrees(180)),
//            new Pose2d(143, 108, Rotation2d.fromDegrees(180)),
//            new Pose2d(32, 113, Rotation2d.fromDegrees(-180))
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
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(40),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(120)
        );
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
