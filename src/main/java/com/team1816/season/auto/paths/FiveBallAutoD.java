package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FiveBallAutoD implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(50, 53, Rotation2d.fromDegrees(90)),
            new Pose2d(132, 156, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(-10));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
