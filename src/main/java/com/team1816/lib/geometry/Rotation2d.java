package com.team1816.lib.geometry;

import com.team254.lib.geometry.Translation2d;

public class Rotation2d extends edu.wpi.first.math.geometry.Rotation2d {

    Rotation2d() {
        super();
    }

    public Rotation2d(double radians) {
        super(radians);
    }

    public Rotation2d(double x, double y) {
        super(x, y);
    }

    protected static final Rotation2d kIdentity = new Rotation2d();

    public Rotation2d(Translation2d slope) {
        super(slope.x(), slope.y());
    }

    public static Rotation2d identity() {
        return kIdentity;
    }

    public Rotation2d inverse() {
        return (Rotation2d) this.unaryMinus();
    }

    public Rotation2d rotateBy(Rotation2d other) {
        return (Rotation2d) super.rotateBy(other);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return (Rotation2d) edu.wpi.first.math.geometry.Rotation2d.fromDegrees(degrees);
    }

    // This seems odd added from 254 maybe remove
    public double distance(final Rotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

}
