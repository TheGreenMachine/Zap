package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FiveBallPathC extends AutoPath {

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(301, 23, Rotation2d.fromDegrees(180)),
            new Pose2d(50, 53, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(225));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
