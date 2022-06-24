package com.team1816.lib.math;

import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.util.Util;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class DriveConversions {

    private static final double azimuthPPR = Constants.Swerve.kAzimuthPPR;
    private static final double wheelCircumferenceInches =
        Constants.kWheelCircumferenceInches;
    private static final double DRIVE_ENCODER_PPR = Drive.DRIVE_ENCODER_PPR;

    // make the degree stuff to ticks/radians/ whatnot come into here instead of being spread out everywhere
    public static double convertTicksToMeters(double ticks) {
        return ticks / DRIVE_ENCODER_PPR * Constants.kWheelCircumferenceMeters;
    }

    public static double convertInchesToTicks(double inches) {
        return inches / Math.PI * azimuthPPR / wheelCircumferenceInches;
    }

    public static double convertMetersToTicks(double meters) {
        return convertInchesToTicks(Units.metersToInches(meters));
    }

    public static double convertDegreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    public static double convertRadiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    public static double convertTicksToRadians(double ticks) {
        return ticks / azimuthPPR * 2 * Math.PI;
    }

    public static double convertRadiansToTicks(double radians) {
        return radians / (Math.PI * 2) * azimuthPPR;
    }

    public static double convertTicksToDegrees(double ticks) {
        return convertRadiansToDegrees(convertTicksToRadians(ticks));
    }

    public static double convertDegreesToTicks(double degrees) {
        return convertRadiansToTicks(convertDegreesToRadians(degrees));
    }

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kWheelCircumferenceInches);
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double metersPerSecondToTicksPer100ms(double meters_per_second) {
        return inchesPerSecondToTicksPer100ms(Units.metersToInches(meters_per_second));
    }

    public static double ticksPerSecondToMetersPer100ms(double ticks_per_second) {
        return ((Units.inchesToMeters(ticksPerSecondToInchesPer100ms(ticks_per_second))));
    }

    public static double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToRotations(inches_per_second) * DRIVE_ENCODER_PPR / 10.0;
    }

    public static double ticksPerSecondToInchesPer100ms(double ticks_per_second) {
        return rotationsToInches(ticks_per_second / DRIVE_ENCODER_PPR) / 10.0;
    }

    public static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    public static boolean epsilonEquals(
        final Pose2d reference,
        final Pose2d other,
        double epsilon
    ) {
        return (
            Util.epsilonEquals(reference.getX(), other.getX(), epsilon) &&
            Util.epsilonEquals(reference.getY(), other.getY(), epsilon) &&
            Util.epsilonEquals(
                reference.getRotation().getDegrees(),
                other.getRotation().getDegrees(),
                epsilon
            )
        );
    }
}
