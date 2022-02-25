package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FiveBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(261, 221, Rotation2d.fromDegrees(130)),
            new Pose2d(223, 265, Rotation2d.fromDegrees(130))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(130),
            Rotation2d.fromDegrees(220)
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
