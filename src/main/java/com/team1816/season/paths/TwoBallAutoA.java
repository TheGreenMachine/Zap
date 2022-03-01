package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TwoBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(247, 208, Rotation2d.fromDegrees(172)),
            new Pose2d(191, 240, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(172), Rotation2d.fromDegrees(180));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
