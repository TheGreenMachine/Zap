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

    public static final SwerveDriveSignal NEUTRAL = new SwerveDriveSignal(ZERO_SPEED, ZERO_AZIMUTH, false);
    public static final SwerveDriveSignal BRAKE = new SwerveDriveSignal(ZERO_SPEED, ZERO_AZIMUTH, true);

    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!
    private boolean mBrakeMode;

    public SwerveDriveSignal() {
        this(ZERO_SPEED, ZERO_AZIMUTH, false);
    }

    public SwerveDriveSignal(double left, double right) {
        super(left, right);
        mWheelSpeeds = new double[4];
//        mWheelSpeeds[Constants.Swerve.kFrontLeft] = left;
//        mWheelSpeeds[Constants.Swerve.kBackLeft] = left;
//        mWheelSpeeds[Constants.Swerve.kFrontRight] = right;
//        mWheelSpeeds[Constants.Swerve.kBackRight] = right;

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
                    x * inchesPerSecondToTicksPer100ms(Constants.kPathFollowingMaxVelMeters)
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
