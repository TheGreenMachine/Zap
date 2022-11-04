package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

/**
 * This class contains all constants pertinent to robot-specific aspects.
 * Only fields that are necessary and generalizable across systems belong in this class.
 */
@Singleton
public class Constants {

    /**
     * Factory & Stem
     */
    private static final RobotFactory factory = Robot.getFactory();

    public static final Pose2d EmptyPose = new Pose2d();
    public static final Rotation2d EmptyRotation = new Rotation2d();
    public static final double kLooperDt = factory.getConstant("kLooperDt", .020);

    /**
     * CAN Timeouts
     */
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    /**
     * Field characterization
     */
    public static final double kTargetHeight = 104; // inches
    public static final double kTargetRadius = 26.56; // inches
    public static final double kCameraMountingHeight = 29.5; // inches
    public static final double kHeightFromCamToHub =
        kTargetHeight - kCameraMountingHeight; // inches
    public static final double kCameraMountingAngleY = 26; // degrees
    public static final double kTurretZedRadius = Units.inchesToMeters(7); // meters TODO: VERIFY VALUE
    public static final double fieldCenterY = 8.23 / 2.0;
    public static final double fieldCenterX = 16.46 / 2.0;
    public static final Pose2d fieldCenterPose = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );
    public static final Pose2d targetPos = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );
    public static final HashMap<Integer, Pose3d> fieldTargets = new HashMap<>() {
        {
            /**
             * Retro-reflective tape targets
             * */
            put(-1, new Pose3d()); // return if vision sees nothing
            /**
             * April Tags
             */

            // blue alliance side
            put(0, new Pose3d(new Translation3d(0.000, 7.510, 0.886), new Rotation3d())); // Blue Hangar Panel

            put(1, new Pose3d(new Translation3d(3.320, 5.588, 1.725), new Rotation3d())); // upper hangar target - Blue Hangar Truss - Hub

            put(2, new Pose3d(new Translation3d(3.072, 5.249, 1.376), new Rotation3d())); // target - Blue Hangar Truss - Side

            put(3, new Pose3d(new Translation3d(0.008, 5.966, 1.376), new Rotation3d())); // Blue Station 2 Wall

            put(4, new Pose3d(new Translation3d(0.008, 3.575, 1.376), new Rotation3d())); // Blue Station 3 Wall

            // blue human player station
            put(5, new Pose3d(new Translation3d(0.125, 1.656, 0.891), new Rotation3d())); // Blue Terminal Near Station

            put(6, new Pose3d(new Translation3d(0.877, 0.879, 0.891), new Rotation3d())); // Blue Mid Terminal

            put(7, new Pose3d(new Translation3d(1.619, 0.095, 0.891), new Rotation3d())); // Blue End Terminal

            // red alliance side
            put(
                10,
                new Pose3d(new Translation3d(16.460, 0.589, 0.886), new Rotation3d())
            ); // Red Hangar Panels

            put(
                11,
                new Pose3d(new Translation3d(13.240, 2.750, 1.725), new Rotation3d())
            ); // upper hangar target

            put(
                12,
                new Pose3d(new Translation3d(13.395, 2.838, 1.376), new Rotation3d())
            ); // lower hangar target

            put(
                13,
                new Pose3d(new Translation3d(16.459, 3.114, 0.806), new Rotation3d())
            );

            put(
                14,
                new Pose3d(new Translation3d(16.459, 4.655, 0.806), new Rotation3d())
            );

            // red human player station
            put(
                15,
                new Pose3d(new Translation3d(16.339, 6.453, 0.894), new Rotation3d())
            );

            put(
                16,
                new Pose3d(new Translation3d(15.594, 7.231, 0.891), new Rotation3d())
            );

            put(
                17,
                new Pose3d(new Translation3d(14.851, 8.007, 0.891), new Rotation3d())
            );

            // lower hub targets
            put(40, new Pose3d(new Translation3d(7.878, 4.851, 0.703), new Rotation3d()));

            put(41, new Pose3d(new Translation3d(7.435, 3.697, 0.703), new Rotation3d()));

            put(42, new Pose3d(new Translation3d(8.589, 3.254, 0.703), new Rotation3d()));

            put(43, new Pose3d(new Translation3d(9.032, 4.408, 0.703), new Rotation3d()));

            // upper hub targets
            put(50, new Pose3d(new Translation3d(7.684, 4.330, 2.408), new Rotation3d()));

            put(51, new Pose3d(new Translation3d(8.020, 3.576, 2.408), new Rotation3d()));

            put(52, new Pose3d(new Translation3d(8.775, 3.912, 2.408), new Rotation3d()));

            put(53, new Pose3d(new Translation3d(8.439, 4.667, 2.408), new Rotation3d()));
        }
    };
    public static final Pose2d kDefaultZeroingPose = new Pose2d(
        0.5,
        fieldCenterY,
        EmptyRotation
    );

    public static final Translation2d kTurretMountingOffset = new Translation2d(
        -.12065,
        .13335
    );

    /**
     * Drivetrain characterization
     */
    public static final double gravitationalAccelerationConstant = 9.8d;
    public static double kMaxAccelDiffThreshold = 2d; // m/s^2

    /**
     * Badlog
     */
    public static boolean kIsBadlogEnabled = factory.getConstant("badLogEnabled") > 0;
    public static boolean kIsLoggingTeleOp = factory.getConstant("logTeleOp") > 0;
    public static boolean kIsLoggingAutonomous = factory.getConstant("logAuto") > 0;

    public final boolean kUsePoseTrack =
        factory.getConstant("shooter", "usingPoseForSpeed", 0) > 0;
    public static final double kBallEjectionDuration = factory.getConstant(
        "shooter",
        "ballEjectionDuration",
        1d
    );
    public static final boolean kUseVision = factory.getSubsystem("camera").implemented;
}
