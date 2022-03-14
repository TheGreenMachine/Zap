package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class OneBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(252, 191, Rotation2d.fromDegrees(336)),
            new Pose2d(222, 243, Rotation2d.fromDegrees(348)),
            new Pose2d(119, 153, Rotation2d.fromDegrees(360)),
            new Pose2d(188, 175, Rotation2d.fromDegrees(360))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(336),
            Rotation2d.fromDegrees(348),
            Rotation2d.fromDegrees(360),
            Rotation2d.fromDegrees(360)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
