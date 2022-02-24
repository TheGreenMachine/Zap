package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FiveBallAutoC implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(313, 24, Rotation2d.fromDegrees(180)),
            new Pose2d(51, 54, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(225)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
