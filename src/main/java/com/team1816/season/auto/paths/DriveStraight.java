package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class DriveStraight implements PathContainer {

    private final int driveDistance;

    public DriveStraight(int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
    }

    public DriveStraight(int driveDistance) {
        this(driveDistance, PathContainer.kMaxVelocity);
    }

    public DriveStraight() {
        this(12);
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d((driveDistance), 0.0, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return null;
    }

    @Override
    public boolean usingApp() {
        return false;
    }
}
