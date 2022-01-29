package com.team1816.lib.geometry;

public class Pose2d extends edu.wpi.first.math.geometry.Pose2d {

    protected static final Pose2d kIdentity = new Pose2d();

    public static Pose2d identity() {
        return kIdentity;
    }

    public Pose2d() {
        super();
    }

    public Pose2d(double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    public Pose2d(double x, double y, double degrees) {
        super(x, y, Rotation2d.fromDegrees(degrees));
    }
}
