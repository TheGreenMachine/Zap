package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TwoBallAutoB implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(300, 97, Rotation2d.fromDegrees(66)),
            new Pose2d(261, 62, Rotation2d.fromDegrees(220)),
            new Pose2d(247, 28, Rotation2d.fromDegrees(360)),
            new Pose2d(347, 19, Rotation2d.fromDegrees(360)),
            new Pose2d(322, 59, Rotation2d.fromDegrees(393)),
            new Pose2d(300, 97, Rotation2d.fromDegrees(426))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(66), Rotation2d.fromDegrees(220), Rotation2d.fromDegrees(360), Rotation2d.fromDegrees(360), Rotation2d.fromDegrees(393), Rotation2d.fromDegrees(426));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
