package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(
            new Pose2d(
                Units.inchesToMeters(79.5),
                Units.inchesToMeters(11.0),
                Rotation2d.fromDegrees(45)
            )
        );
        waypoints.add(
            new Pose2d(
                Units.inchesToMeters(172),
                Units.inchesToMeters(30),
                Rotation2d.fromDegrees(0)
            )
        );
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
