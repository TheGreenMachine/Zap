package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TwoBallAutoB implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(271, 96, Rotation2d.fromDegrees(220)),
            new Pose2d(199, 74, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(220), Rotation2d.fromDegrees(180));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
