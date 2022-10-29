package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    public static final Transform2d fieldCenterTransform = new Transform2d(
        new Pose2d(),
        fieldCenterPose
    );
    public static final Pose2d targetPos = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );
    public static final HashMap<Integer, Double[]> fieldTargets = new HashMap<>() {
        {
            /**
             * Retro-reflective tape targets
             * */
            put(
                -1,
                new Double[]{
                    Constants.fieldCenterX,
                    Constants.fieldCenterY,
                    Units.inchesToMeters(Constants.kTargetHeight),
                    1.0,
                }
            ); // center hub
            /**
             * April Tags
             */

            // blue alliance side
            put(00, new Double[] {0.000, 7.510, 0.886});

            put(01, new Double[] {3.270, 5.588, 1.725}); // upper hangar target

            put(02, new Double[] {3.072, 5.249, 1.376}); // lower hangar target

            put(03, new Double[] {0.008, 5.116, 1.376});

            put(04, new Double[] {0.008, 3.575, 1.376});

            // blue human player station
            put(05, new Double[] {0.125, 1.656, 0.891});

            put(06, new Double[] {0.877, 0.879, 0.891});

            put(07, new Double[] {1.619, 0.095, 0.891});

            // red alliance side
            put(10, new Double[]{16.467, 0.589, 0.886});

            put(11, new Double[]{13.240, 2.750, 1.725}); // upper hangar target

            put(12, new Double[]{13.395, 2.838, 1.376}); // lower hangar target

            put(13, new Double[]{16.459, 3.114, 0.806});

            put(14, new Double[]{16.459, 4.655, 0.806});

            // red human player station
            put(15, new Double[]{16.339, 6.453, 0.894});

            put(16, new Double[]{15.594, 7.231, 0.891});

            put(17, new Double[]{14.851, 8.007, 0.891});

            // lower hub targets
            put(40, new Double[]{7.878, 4.851, 0.703});

            put(41, new Double[]{7.435, 3.697, 0.703});

            put(42, new Double[]{8.589, 3.254, 0.703});

            put(43, new Double[]{9.032, 4.408, 0.703});

            // upper hub targets
            put(50, new Double[]{7.684, 4.330, 2.408});

            put(51, new Double[]{8.020, 3.576, 2.408});

            put(52, new Double[]{8.775, 3.912, 2.408});

            put(53, new Double[]{8.439, 4.667, 2.408});
        }
    };
    public static final Pose2d kDefaultZeroingPose = new Pose2d(
        0.5,
        fieldCenterY,
        EmptyRotation
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
