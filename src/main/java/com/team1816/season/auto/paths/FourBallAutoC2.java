package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FourBallAutoC2 implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(299, 28, Rotation2d.fromDegrees(-179)),
            new Pose2d(55, 55, Rotation2d.fromDegrees(-189))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(-179), Rotation2d.fromDegrees(-160));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
