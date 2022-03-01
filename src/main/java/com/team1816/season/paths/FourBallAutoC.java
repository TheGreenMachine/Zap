package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FourBallAutoC implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(330, 72, Rotation2d.fromDegrees(270)),
            new Pose2d(317, 17, Rotation2d.fromDegrees(180)),
            new Pose2d(200, 75, Rotation2d.fromDegrees(180)),
            new Pose2d(72, 52, Rotation2d.fromDegrees(160)),
            new Pose2d(110, 157, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(270),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(160),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
