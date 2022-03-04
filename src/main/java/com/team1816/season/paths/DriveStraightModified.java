package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class DriveStraightModified implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(237, 192, Rotation2d.fromDegrees(0)),
            new Pose2d(231, 157, Rotation2d.fromDegrees(180)),
            new Pose2d(156, 153, Rotation2d.fromDegrees(160)),
            new Pose2d(68, 144, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(160), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
