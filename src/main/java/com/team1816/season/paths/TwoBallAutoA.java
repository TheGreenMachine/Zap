package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TwoBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(235, 207, Rotation2d.fromDegrees(135)),
            new Pose2d(198, 243, Rotation2d.fromDegrees(135))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(135),
            Rotation2d.fromDegrees(135)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
