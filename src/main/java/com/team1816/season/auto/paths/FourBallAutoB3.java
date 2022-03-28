package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FourBallAutoB3 implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(60, 60, Rotation2d.fromDegrees(0)), // -160
            new Pose2d(198, 77, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(-100), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
