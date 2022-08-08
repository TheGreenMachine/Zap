package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FourBallPathB2 extends AutoPath {

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(199, 79, Rotation2d.fromDegrees(-155)),
            new Pose2d(45, 55, Rotation2d.fromDegrees(-160))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(Rotation2d.fromDegrees(-145), Rotation2d.fromDegrees(-130));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
