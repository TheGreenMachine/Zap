package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class ThreeBallAutoC implements PathContainer {
    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(298, 75, Rotation2d.fromDegrees(180)),
            new Pose2d(198, 76, Rotation2d.fromDegrees(200)),
            new Pose2d(52, 53, Rotation2d.fromDegrees(135)),
            new Pose2d(136, 164, Rotation2d.fromDegrees(-20))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(200), Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-20));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
