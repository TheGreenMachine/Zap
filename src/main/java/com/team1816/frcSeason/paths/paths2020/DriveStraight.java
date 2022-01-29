package com.team1816.frcSeason.paths.paths2020;

import com.team1816.lib.paths.PathContainer;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team1816.lib.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class DriveStraight implements PathContainer {

    private final int driveDistance;
    private final double maxVel;

    public DriveStraight(int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
        this.maxVel = maxVel;
    }

    public DriveStraight(int driveDistance) {
        this(driveDistance, PathContainer.kMaxVelocity);
    }

    public DriveStraight() {
        this(12);
    }

    @Override
    public Path buildPath() {
        /*
      ArrayList<PathBuilder.Waypoint> waypoints = new ArrayList<>();
      waypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0);
      waypoints.add(new PathBuilder.Waypoint(driveDistance, 0,0, 0));
      return  PathBuilder.buildPathFromWaypoints(waypoints);
       */
        return new Path();
    }

    @Override
    public List<Pose2d> buildWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        var heading = (driveDistance < 0 ? 180 : 0);
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(driveDistance, 0.0, Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
    public boolean isReversed() {
        //        return false;
        return driveDistance < 0;
    }

    @Override
    public double getMaxVelocity() {
        return maxVel;
    }
}
