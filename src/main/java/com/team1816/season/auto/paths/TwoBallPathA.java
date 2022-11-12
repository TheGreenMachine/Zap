package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class TwoBallPathA extends AutoPath {

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(245, 202, Rotation2d.fromDegrees(135)),
            new Pose2d(209, 236, Rotation2d.fromDegrees(135))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(135));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
