//package com.team1816.season.paths;
//
//import com.team1816.lib.paths.PathContainer;
//import com.team254.lib.control.Path;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//
//import java.util.List;
//
//public class FiveBallAutoB implements PathContainer {
//
//    @Override
//    public List<edu.wpi.first.math.geometry.Pose2d> buildWaypoints() {
//        return List.of(
//            new Pose2d(55, 55, Rotation2d.fromDegrees(0)),
//            new Pose2d(191, 67, Rotation2d.fromDegrees(0)),
//            new Pose2d(297, 24, Rotation2d.fromDegrees(0))
//        );
//    }
//
//    @Override
//    public List<Rotation2d> buildHeadings() {
//        return List.of(
//            Rotation2d.fromDegrees(0),
//            Rotation2d.fromDegrees(0),
//            Rotation2d.fromDegrees(0)
//        );
//    }
//
//    @Override
//    public boolean isReversed() {
//        return false;
//    }
//}
