package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class DriveStraight extends AutoPath {

    private final int driveDistance;

    public DriveStraight(int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
    }

    public DriveStraight(int driveDistance) {
        this(driveDistance, Constants.kPathFollowingMaxVelMeters);
    }

    public DriveStraight() {
        this(12);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d((driveDistance), 0.0, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return null;
    }

    @Override
    public boolean usingApp() {
        return false;
    }
}
