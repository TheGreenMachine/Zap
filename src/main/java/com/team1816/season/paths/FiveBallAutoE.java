package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FiveBallAutoE implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(249, 210, Rotation2d.fromDegrees(135)),
            new Pose2d(178, 253, Rotation2d.fromDegrees(-90)),
            new Pose2d(61, 59, Rotation2d.fromDegrees(-31)),
            new Pose2d(201, 74, Rotation2d.fromDegrees(0)),
            new Pose2d(307, 14, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(135),
            Rotation2d.fromDegrees(270),
            Rotation2d.fromDegrees(329),
            Rotation2d.fromDegrees(360),
            Rotation2d.fromDegrees(360)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
