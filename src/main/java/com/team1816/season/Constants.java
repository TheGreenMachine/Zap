package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
    public static final double fieldCenterY = 8.23 / 2.0;
    public static final double fieldCenterX = 16.46 / 2.0;
    public static final Pose2d targetPos = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );

    // Drivetrain characterization
    public static final double kDriveWheelTrackWidthInches = factory.getConstant(
        "trackWidth",
        22
    );
    public static final double kDriveWheelbaseLengthInches = factory.getConstant(
        "wheelbaseLength",
        22
    );
    public static final double kDriveWheelDiameterInches = factory.getConstant(
        "wheelDiameter"
    );
    public static final double kWheelCircumferenceInches =
        kDriveWheelDiameterInches * Math.PI;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;

    public static final double kDriveWheelTrackWidthMeters = Units.inchesToMeters(
        kDriveWheelTrackWidthInches
    );
    public static final double kDriveWheelbaseLengthMeters = Units.inchesToMeters(
        kDriveWheelbaseLengthInches
    );
    public static final double kDriveWheelDiameterMeters = Units.inchesToMeters(
        kDriveWheelDiameterInches
    );
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(
        kWheelCircumferenceInches
    );
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(
        kDriveWheelRadiusInches
    );
    public static double kTrackScrubFactor = factory.getConstant("kTrackScrubFactor");

    public static final Pose2d ZeroPose = new Pose2d(0.5, fieldCenterY, EmptyRotation);
    public static Pose2d StartingPose = new Pose2d(0.5, fieldCenterY, EmptyRotation);

    public static class Tank {

        public String kName = "Name";

        public static final DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(
            kDriveWheelTrackWidthMeters
        );
    }

    public static class Swerve {

        public String kModuleName = "Name";
        public String kDriveMotorName = "";
        public String kAzimuthMotorName = "";

        public PIDSlotConfiguration kAzimuthPid;
        public PIDSlotConfiguration kDrivePid;

        // constants defined for each swerve module
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public double kAzimuthEncoderHomeOffset;

        public static final int kAzimuthPPR = (int) factory.getConstant(
            "drive",
            "azimuthEncPPR",
            4096
        );

        // azimuth position
        private static final double moduleDeltaX = kDriveWheelbaseLengthMeters / 2.0;
        private static final double moduleDeltaY = kDriveWheelTrackWidthMeters / 2.0;

        // Module Indicies
        public static final int kFrontLeft = 0;
        public static final int kFrontRight = 1;
        public static final int kBackLeft = 2;
        public static final int kBackRight = 3;

        public static final Translation2d kFrontLeftModulePosition = new Translation2d(
            moduleDeltaX,
            moduleDeltaY
        );
        public static final Translation2d kFrontRightModulePosition = new Translation2d(
            moduleDeltaX,
            -moduleDeltaY
        );
        public static final Translation2d kBackLeftModulePosition = new Translation2d(
            -moduleDeltaX,
            moduleDeltaY
        );
        public static final Translation2d kBackRightModulePosition = new Translation2d(
            -moduleDeltaX,
            -moduleDeltaY
        );

        public static final Translation2d[] kModulePositions = {
            kFrontLeftModulePosition,
            kFrontRightModulePosition,
            kBackRightModulePosition,
            kBackLeftModulePosition,
        };

        // See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            kFrontLeftModulePosition,
            kFrontRightModulePosition,
            kBackLeftModulePosition,
            kBackRightModulePosition
        );
    }

    // Control Board
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorGamepadPort = 1;
    public static final double kJoystickThreshold = 0.04; // deadband

    // Drive speed
    public static final double kPathFollowingMaxAccelMeters = factory.getConstant(
        "maxAccel",
        4
    );
    public static final double kPathFollowingMaxVelMeters = factory.getConstant(
        "maxVelPathFollowing"
    );
    public static final double kOpenLoopMaxVelMeters = factory.getConstant(
        "maxVelOpenLoop"
    );

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 4;
    public static final double kMaxAngularSpeed = factory.getConstant("maxRotVel"); // rad/sec
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
        2 * Math.PI;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared
    );

    //Badlog
    public static final boolean kIsBadlogEnabled =
        factory.getConstant("badLogEnabled") > 0;
    public static final boolean kIsLoggingTeleOp = factory.getConstant("logTeleOp") > 0;
    public static final boolean kIsLoggingAutonomous = factory.getConstant("logAuto") > 0;

    public static final boolean kUsePoseTrack =
        factory.getConstant("shooter", "usingPoseForSpeed", 0) > 0;
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
