package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class ThreeBallAutoB implements PathContainer {
    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(269, 99, Rotation2d.fromDegrees(-135)),
            new Pose2d(202, 74, Rotation2d.fromDegrees(-135)),
            new Pose2d(230, 18, Rotation2d.fromDegrees(-15)),
            new Pose2d(317, 16, Rotation2d.fromDegrees(15))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(-135), Rotation2d.fromDegrees(-115), Rotation2d.fromDegrees(-15), Rotation2d.fromDegrees(15));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}

