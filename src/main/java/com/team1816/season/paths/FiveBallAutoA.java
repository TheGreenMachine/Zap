//package com.team1816.season.paths;
//
//import com.team1816.lib.paths.PathContainer;
//import com.team254.lib.control.Path;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//
//import java.util.List;
//
//public class FiveBallAutoA implements PathContainer {
//
//    @Override
//    public List<Pose2d> buildWaypoints() {
//        return List.of(
//            new Pose2d(268, 212, Rotation2d.fromDegrees(130)),
//            new Pose2d(204, 235, Rotation2d.fromDegrees(-140)),
//            new Pose2d(55, 55, Rotation2d.fromDegrees(-170))
//        );
//    }
//
//    @Override
//    public List<Rotation2d> buildHeadings() {
//        return List.of(
//            Rotation2d.fromDegrees(130),
//            Rotation2d.fromDegrees(-140),
//            Rotation2d.fromDegrees(-170)
//        );
//    }
//
//    @Override
//    public boolean isReversed() {
//        return false;
//    }
//}
