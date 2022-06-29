package com.team254.lib.util;

import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

import java.text.DecimalFormat;
import java.util.Arrays;

import static com.team1816.lib.math.DriveConversions.inchesPerSecondToTicksPer100ms;

/**
 * A drivetrain signal containing the speed and azimuth for each wheel
 */
public class SwerveDriveSignal extends DriveSignal {
    public static final double[] ZERO_SPEED = new double[]{0, 0, 0, 0};
    public static final Rotation2d[] ZERO_AZIMUTH = new Rotation2d[]{new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};
    public static final Rotation2d[] X_AZIMUTH = new Rotation2d[]{
        Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(45)
    };
    public static final Rotation2d[] AZIMUTH_90_DEGREES = new Rotation2d[]{
        Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)
    };

    public static final SwerveDriveSignal NEUTRAL = new SwerveDriveSignal(ZERO_SPEED, ZERO_AZIMUTH, false);
    public static final SwerveDriveSignal BRAKE = new SwerveDriveSignal(ZERO_SPEED, X_AZIMUTH, false);


    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!
    private boolean mBrakeMode;

    public SwerveDriveSignal() {
        this(ZERO_SPEED, ZERO_AZIMUTH, false);
    }

    public SwerveDriveSignal(double left, double right) {
        super(left, right);
        mWheelSpeeds = new double[4];
        mWheelAzimuths = ZERO_AZIMUTH;
        mBrakeMode = false;
    }

    public SwerveDriveSignal(double[] wheelSpeeds, Rotation2d[] wheelAzimuths, boolean brakeMode) {
        super(
            wheelSpeeds[Constants.Swerve.kFrontLeft],
            wheelSpeeds[Constants.Swerve.kFrontRight]
        );
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
        mBrakeMode = brakeMode;
    }

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

    public SwerveDriveSignal toVelocity() {
        return new SwerveDriveSignal(
            Arrays.stream(this.mWheelSpeeds)
                .map(x ->
                    x * inchesPerSecondToTicksPer100ms(Constants.kOpenLoopMaxVelMeters)
                )
                .toArray(),
            this.mWheelAzimuths,
            this.mBrakeMode
        );
    }


    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        String ret_val = "DriveSignal - \n";
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        for (int i = 0; i < mWheelSpeeds.length; i++) {
            ret_val += "\tWheel " + i + ": Speed - " + mWheelSpeeds[i] + ", Azimuth - " + fmt.format(mWheelAzimuths[i].getDegrees()) + " deg\n";
        }

        return ret_val;
    }
}
