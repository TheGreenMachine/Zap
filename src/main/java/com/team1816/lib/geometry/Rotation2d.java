package com.team1816.lib.geometry;


public class Rotation2d extends edu.wpi.first.math.geometry.Rotation2d {

    protected static final Rotation2d kIdentity = new Rotation2d();

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
}
