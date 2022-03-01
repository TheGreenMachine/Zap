package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FourBallSemiCircleAutoB implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(320, 82, Rotation2d.fromDegrees(270)),
            new Pose2d(295, 22, Rotation2d.fromDegrees(180)),
            new Pose2d(199, 77, Rotation2d.fromDegrees(120)),
            new Pose2d(254, 259, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(270), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
