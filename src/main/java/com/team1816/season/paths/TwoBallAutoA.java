package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TwoBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(252, 191, Rotation2d.fromDegrees(-24)),
            new Pose2d(58, 156, Rotation2d.fromDegrees(-24)),
            new Pose2d(105, 253, Rotation2d.fromDegrees(-24)),
            new Pose2d(191, 253, Rotation2d.fromDegrees(-24)),
            new Pose2d(252, 191, Rotation2d.fromDegrees(-24))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(-24), Rotation2d.fromDegrees(-24), Rotation2d.fromDegrees(-24), Rotation2d.fromDegrees(-24), Rotation2d.fromDegrees(-24));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
