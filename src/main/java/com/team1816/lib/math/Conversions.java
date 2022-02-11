package com.team1816.lib.math;

import com.team1816.season.Constants;
import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;

public class Conversions {
    private static final double ticksPerRevolution = 4096;
    private static final double metersPerInch = 0.0254;
    private static final double wheelCircumferenceInches = Constants.kWheelCircumferenceInches;

    // make the degree stuff to ticks/radians/ whatnot come into here instead of being spread out everywhere
    public static double convertInchesToMeters(double inches) {
        return inches*metersPerInch;
    }

    public static double convertMetersToInches(double meters) {
        return meters/metersPerInch;
    }

    public static double convertTicksToInches(double ticks) {
        return ticks/ticksPerRevolution*wheelCircumferenceInches*Math.PI;
    }

    public static double convertInchesToTicks(double inches) {
        return inches/Math.PI*ticksPerRevolution/wheelCircumferenceInches;
    }

    public static double convertTicksToMeters(double ticks) {
        return convertInchesToMeters(convertTicksToInches(ticks));
    }

    public static double convertMetersToTicks(double meters) {
        return convertInchesToTicks(convertMetersToInches(meters));
    }

    public static double convertDegreesToRadians(double degrees) {
        return degrees*Math.PI/180;
    }

    public static double convertRadiansToDegrees(double radians) {
        return radians*180/Math.PI;
    }

    public static double convertTicksToRadians(double ticks) {
        return ticks/ticksPerRevolution*2*Math.PI;
    }

    public static double convertRadiansToTicks(double radians) {
        return radians/(Math.PI*2)*ticksPerRevolution;
    }

    public static double convertTicksToDegrees(double ticks) {
        return convertRadiansToDegrees(convertTicksToRadians(ticks));
    }

    public static double convertDegreesToTicks(double degrees) {
        return convertRadiansToTicks(convertDegreesToRadians(degrees));
    }

    public static boolean epsilonEquals(final Pose2d reference, final Pose2d other, double epsilon) {
        return Util.epsilonEquals(reference.getX(), other.getX(), epsilon)
            && Util.epsilonEquals(reference.getY(), other.getY(), epsilon)
            && Util.epsilonEquals(reference.getRotation().getDegrees(), other.getRotation().getDegrees(), epsilon
        );
    }

}
