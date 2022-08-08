package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FourBallPathB3 extends AutoPath {

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(55, 55, Rotation2d.fromDegrees(0)), // -160
            new Pose2d(198, 77, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(Rotation2d.fromDegrees(-100), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
