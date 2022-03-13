package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FiveBallAutoB implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(264, 228, Rotation2d.fromDegrees(135)),
            new Pose2d(225, 267, Rotation2d.fromDegrees(230)),
            new Pose2d(225, 267, Rotation2d.fromDegrees(230)),
            new Pose2d(310, 24, Rotation2d.fromDegrees(0)),
            new Pose2d(58, 34, Rotation2d.fromDegrees(180)),
            new Pose2d(58, 34, Rotation2d.fromDegrees(180)),
            new Pose2d(140, 155, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(130),
            Rotation2d.fromDegrees(220),
            Rotation2d.fromDegrees(220),
            Rotation2d.fromDegrees(360),
            Rotation2d.fromDegrees(225),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(-10)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
