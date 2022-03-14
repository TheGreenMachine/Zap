package com.team1816.season.paths;

import com.team1816.lib.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class OneBallAutoB implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(300, 97, Rotation2d.fromDegrees(66)),
            new Pose2d(274, 30, Rotation2d.fromDegrees(180)),
            new Pose2d(179, 29, Rotation2d.fromDegrees(160)),
            new Pose2d(54, 143, Rotation2d.fromDegrees(70)),
            new Pose2d(175, 166, Rotation2d.fromDegrees(-30)) //,
            //new Pose2d(188, 175, Rotation2d.fromDegrees(-30))

        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(
            Rotation2d.fromDegrees(66),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(160),
            Rotation2d.fromDegrees(70),
            Rotation2d.fromDegrees(-50)
            /*, Rotation2d.fromDegrees(-30)*/
        );
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
