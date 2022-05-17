package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class OneBallAutoC_Border implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(334, 75, Rotation2d.fromDegrees(-66)),
            new Pose2d(352, 26, Rotation2d.fromDegrees(-66))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(-66), Rotation2d.fromDegrees(-66));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
