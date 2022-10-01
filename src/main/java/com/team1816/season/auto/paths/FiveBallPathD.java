package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.List;

public class FiveBallPathD extends AutoPath {

    boolean color = false;

    public FiveBallPathD() {}

    public FiveBallPathD(boolean color) {
        this.color = color;
    }

    @Override
    public List<Pose2d> getWaypoints() {
        var waypoints = Arrays.asList(
            new Pose2d(50, 53, Rotation2d.fromDegrees(90)),
            new Pose2d(132, 156, Rotation2d.fromDegrees(0))
        );
        if (color) {
            for (int i = 0; i < waypoints.size(); i++) {
                var point = waypoints.get(i);
                waypoints.set(
                    i,
                    new Pose2d(
                        648 - point.getX(),
                        324 - point.getY(),
                        point.getRotation().rotateBy(Rotation2d.fromDegrees(180))
                    )
                );
            }
        }
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        var headings = Arrays.asList(
            Rotation2d.fromDegrees(120),
            Rotation2d.fromDegrees(-10)
        );
        if (color) {
            for (int i = 0; i < headings.size(); i++) {
                headings.set(i, headings.get(i).rotateBy(Rotation2d.fromDegrees(180)));
            }
        }
        return headings;
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
