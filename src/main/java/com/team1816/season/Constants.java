package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

@Singleton
public class Constants {

    private static final RobotFactory factory = Robot.getFactory();

    public static final Pose2d EmptyPose = new Pose2d();
    public static final Rotation2d EmptyRotation = new Rotation2d();
    public static final double kLooperDt = factory.getConstant("kLooperDt", .020);

    // CAN Timeouts
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Field characterization
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
            put(
                -1,
                new Double[] {
                    fieldCenterX,
                    fieldCenterY,
                    Units.inchesToMeters(kTargetHeight),
                }
            ); // retro-reflective tape
        }
    };
    public static final Pose2d kDefaultZeroingPose = new Pose2d(
        0.5,
        fieldCenterY,
        EmptyRotation
    );

    // Drivetrain characterization
    public static final double gravitationalAccelerationConstant = 9.8d;
    public static double kMaxAccelDiffThreshold = 2d; // m/s^2

    //Badlog
    public static boolean kIsBadlogEnabled = factory.getConstant("badLogEnabled") > 0;
    public static boolean kIsLoggingTeleOp = factory.getConstant("logTeleOp") > 0;
    public static boolean kIsLoggingAutonomous = factory.getConstant("logAuto") > 0;

    public final boolean kUsePoseTrack =
        factory.getConstant("shooter", "usingPoseForSpeed", 0) > 0;
    public static final double kBallEjectionDuration = factory.getConstant(
        "shooter",
        "ballEjectionDuration",
        0.5
    );
    public static final boolean kUseVision = factory.getSubsystem("camera").implemented;
    //    public static final boolean kEnableBucketTuning =
    //        factory.getConstant("enableBucketTuning", 0) > 0;
    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

}
