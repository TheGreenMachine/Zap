package com.team1816.frcSeason.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.frcSeason.Constants;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.ISwerveModule;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule extends Subsystem implements ISwerveModule {

    public static class PeriodicIO {

        // INPUTS
        public double drive_encoder_ticks;
        public double azimuth_encoder_ticks; // actual position of module in encoder units, adjusted for home offset
        public double azimuth_encoder_ticks_unmasked;
        public int position_ticks;
        public double velocity_ticks_per_100ms;

        // OUTPUTS
        public double drive_demand;
        public double azimuth_demand; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP,
        VELOCITY,
    }

    public static class SwerveModuleConstants {

        public String kName = "Name";
        public String kDriveMotorName = "";
        public String kAzimuthMotorName = "";

        // general azimuth
        public boolean kInvertAzimuth = false;
        public boolean kInvertAzimuthSensorPhase = false;
        public NeutralMode kAzimuthInitNeutralMode = NeutralMode.Brake; // neutral mode could change
        public double kAzimuthTicksPerRadian = 4096.0 / (2 * Math.PI); // for azimuth
        public double kAzimuthEncoderHomeOffset = 0;
        public double kAzimuthAdjustmentOffset;

        // azimuth motion
        public PIDSlotConfiguration kAzimuthPid;
        public int kAzimuthClosedLoopAllowableError = (int) factory.getConstant(
            "drivetrain",
            "azimuthAllowableErrorTicks"
        );

        // azimuth current/voltage
        public VelocityMeasPeriod kAzimuthVelocityMeasurementPeriod =
            VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms

        // general drive
        public PIDSlotConfiguration kDrivePid;
        public double kWheelDiameter = Constants.kDriveWheelDiameterInches; // Probably should tune for each individual wheel maybe
        public double kDriveTicksPerUnitDistance =
            (1 / Drive.DRIVE_ENCODER_PPR) * (Math.PI * kWheelDiameter);
        public double kDriveDeadband = 0.01;

        // drive current/voltage

        // drive measurement
        public VelocityMeasPeriod kDriveVelocityMeasurementPeriod =
            VelocityMeasPeriod.Period_100Ms; // dt for velocity measurements, ms
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;
    public static CANCoder mCanCoder;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private boolean isBrakeMode = false;
    private double driveEncoderSimPosition = 0;
    private double previousEncDistance = 0;
    private Translation2d position = Translation2d.identity();
    private final Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();
    private final boolean driveMotorIsInverted;

    // Constants
    private final SwerveModuleConstants mConstants;
    private static final double TICK_RATIO_PER_LOOP = Constants.kLooperDt / 0.1;
    public static final int AZIMUTH_TICK_MASK = 0xFFF;
    public static final double AZIMUTH_ADJUSTMENT_OFFSET_DEGREES = factory.getConstant(
        "drive",
        "azimuthHomeAdjustmentDegrees",
        0
    );

    public static final int kFrontLeft = 0;
    public static final int kFrontRight = 1;
    public static final int kBackRight = 2;
    public static final int kBackLeft = 3;

    public SwerveModule(
        String subsystemName,
        SwerveModuleConstants constants,
        Translation2d startingPosition
    ) {
        super(constants.kName);
        mConstants = constants;
        System.out.println(
            "Configuring Swerve Module " +
            constants.kName +
            " on subsystem " +
            subsystemName
        );

        mDriveMotor =
            factory.getMotor(
                subsystemName,
                constants.kDriveMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID
            );
        driveMotorIsInverted = mDriveMotor.getInverted();
        mAzimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.drivePID
            );
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            25,
            0,
            0
        );

        mAzimuthMotor.configSupplyCurrentLimit(currentLimitConfig, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.setSensorPhase(constants.kInvertAzimuthSensorPhase);
        mAzimuthMotor.configPeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.configPeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.setNeutralMode(NeutralMode.Brake);
        mAzimuthMotor.configAllowableClosedloopError(
            0,
            constants.kAzimuthClosedLoopAllowableError,
            Constants.kLongCANTimeoutMs
        );
        System.out.println("  " + this);

        this.startingPosition = startingPosition;
        zeroSensors();
    }

    public SwerveModule(
        String subsystemName,
        SwerveModuleConstants constants,
        CANCoder canCoder,
        Translation2d startingPosition
    ) {
        super(constants.kName);
        mConstants = constants;
        System.out.println(
            "Configuring Swerve Module " +
                constants.kName +
                " on subsystem " +
                subsystemName
        );

        mDriveMotor =
            factory.getMotor(
                subsystemName,
                constants.kDriveMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID
            );
        driveMotorIsInverted = mDriveMotor.getInverted();
        mAzimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.drivePID
            );
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            25,
            0,
            0
        );

        mAzimuthMotor.configSupplyCurrentLimit(currentLimitConfig, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.setSensorPhase(constants.kInvertAzimuthSensorPhase);
        mAzimuthMotor.configPeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.configPeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.setNeutralMode(NeutralMode.Brake);
        mAzimuthMotor.configAllowableClosedloopError(
            0,
            constants.kAzimuthClosedLoopAllowableError,
            Constants.kLongCANTimeoutMs
        );
        mCanCoder = canCoder;

        System.out.println("  " + this);

        this.startingPosition = startingPosition;
        zeroSensors();
    }

    public synchronized void updatePose(Rotation2d robotHeading) {
        double currentEncDistance = getDriveDistance();
        double deltaEncDistance = (currentEncDistance - previousEncDistance);
        Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
        Translation2d deltaPosition = new Translation2d(
            currentWheelAngle.cos() * deltaEncDistance,
            currentWheelAngle.sin() * deltaEncDistance
        );

        //        double xScrubFactor = Constants.kXScrubFactor;
        //        double yScrubFactor = Constants.kYScrubFactor;
        //
        //        if (Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)) {
        //            if (standardCarpetDirection) {
        //                xScrubFactor = 1.0;
        //            } else {
        //
        //            }
        //        } else {
        //            if (standardCarpetDirection) {
        //
        //            } else {
        //                xScrubFactor = 1.0;
        //            }
        //        }
        //        if (Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)) {
        //            if (standardCarpetDirection) {
        //                yScrubFactor = 1.0;
        //            } else {
        //
        //            }
        //        } else {
        //            if (standardCarpetDirection) {
        //
        //            } else {
        //                yScrubFactor = 1.0;
        //            }
        //        }

        deltaPosition =
            new Translation2d(
                deltaPosition.x()/* * xScrubFactor */,
                deltaPosition.y()
                /* * yScrubFactor */
            );
        Translation2d updatedPosition = position.translateBy(deltaPosition);
        Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
        Pose2d robotPose = staticWheelPose.transformBy(
            Pose2d.fromTranslation(startingPosition).inverse()
        );
        position = updatedPosition;
        estimatedRobotPose = robotPose;
        previousEncDistance = currentEncDistance;
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
        Rotation2d normalizedAngle = getAngleToDegrees();
        return normalizedAngle.rotateBy(robotHeading);
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public synchronized void resetPose(Pose2d robotPose) {
        Translation2d modulePosition = robotPose
            .transformBy(Pose2d.fromTranslation(startingPosition))
            .getTranslation();
        position = modulePosition;
    }

    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.drive_demand = speed;
        mPeriodicIO.azimuth_demand = (int) radiansToEncoderUnits(azimuth.getRadians());
    }

    public synchronized void setVelocity(double speed, Rotation2d azimuth) {
        if (mControlState != ControlState.VELOCITY) {
            mControlState = ControlState.VELOCITY;
        }

        mPeriodicIO.drive_demand = speed;
        var isFront = mConstants.kName.startsWith("front");
        var sign = isFront ? 1 : -1;
        var azimuthAdjustmentRadians =
            sign * Math.toRadians(AZIMUTH_ADJUSTMENT_OFFSET_DEGREES);
        mPeriodicIO.azimuth_demand =
            (int) radiansToEncoderUnits(azimuth.getRadians() + azimuthAdjustmentRadians);
    }

    @Override
    public void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            driveEncoderSimPosition += mPeriodicIO.drive_demand * TICK_RATIO_PER_LOOP;
            mPeriodicIO.drive_encoder_ticks = driveEncoderSimPosition;
            mPeriodicIO.velocity_ticks_per_100ms = mPeriodicIO.drive_demand;
            mPeriodicIO.azimuth_encoder_ticks = mPeriodicIO.azimuth_demand;
        } else {
            mPeriodicIO.drive_encoder_ticks = mDriveMotor.getSelectedSensorPosition(0);
            mPeriodicIO.velocity_ticks_per_100ms =
                mDriveMotor.getSelectedSensorVelocity(0);

            var normalizedEncoderTicks = (int) (
                mAzimuthMotor.getSelectedSensorPosition(0) -
                mConstants.kAzimuthEncoderHomeOffset
            );

            mPeriodicIO.azimuth_encoder_ticks_unmasked = normalizedEncoderTicks;
            mPeriodicIO.azimuth_encoder_ticks =
                normalizedEncoderTicks & AZIMUTH_TICK_MASK;
        }
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        mDriveMotor.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (
                Util.epsilonEquals(
                    mPeriodicIO.drive_demand,
                    0.0,
                    mConstants.kDriveDeadband
                )
            ) { // don't move if
                // throttle is 0
                stop();
            } else {
                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
            }
        } else if (mControlState == ControlState.VELOCITY) {
            //            System.out.println(mConstants.kName + " drive demand: " + mPeriodicIO.drive_demand);
            mDriveMotor.set(ControlMode.Velocity, mPeriodicIO.drive_demand);
        }

        double demandedPosition;
        var upDistance = mPeriodicIO.azimuth_demand - mPeriodicIO.azimuth_encoder_ticks;
        if (mPeriodicIO.azimuth_demand < mPeriodicIO.azimuth_encoder_ticks) {
            upDistance += 4096;
        }

        var downDistance = upDistance - 4096;

        if (Math.abs(upDistance) < Math.abs(downDistance)) {
            demandedPosition = mPeriodicIO.azimuth_encoder_ticks_unmasked + upDistance;
        } else {
            demandedPosition = mPeriodicIO.azimuth_encoder_ticks_unmasked + downDistance;
        }

        var offsetDemand =
            ((int) (demandedPosition + mConstants.kAzimuthEncoderHomeOffset));

        mAzimuthMotor.set(ControlMode.Position, offsetDemand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                    synchronized (SwerveModule.this) {
                        stop();
                    }
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (SwerveModule.this) {
                        switch (mControlState) {
                            case OPEN_LOOP:
                                break;
                            case VELOCITY:
                                break;
                            default:
                                System.out.println(
                                    "Unexpected control state: " + mControlState
                                );
                                break;
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    @Override
    public void zeroSensors() {
        if (mAzimuthMotor instanceof TalonSRX) {
            var sensors = ((TalonSRX) mAzimuthMotor).getSensorCollection();
            sensors.setQuadraturePosition(
                sensors.getPulseWidthPosition() & AZIMUTH_TICK_MASK,
                Constants.kLongCANTimeoutMs
            );
        }
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }

    private int getAzimuthPosAbsolute() {
        if (mAzimuthMotor instanceof TalonSRX) {
            int rawValue =
                ((TalonSRX) mAzimuthMotor).getSensorCollection().getPulseWidthPosition() &
                AZIMUTH_TICK_MASK;
            return rawValue;
        }
        return 0;
    }

    @Override
    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        // boolean driveMotorPassed = EnhancedMotorChecker.checkMotors(
        //     this,
        //     EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mDriveMotor),
        //     new EnhancedMotorChecker.NamedMotor(mConstants.kDriveMotorName, mDriveMotor)
        // );
        // boolean azimuthMotorPassed = EnhancedMotorChecker.checkMotors(
        //     this,
        //     EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, mAzimuthMotor),
        //     new EnhancedMotorChecker.NamedMotor(
        //         mConstants.kAzimuthMotorName,
        //         mAzimuthMotor
        //     )
        // );
        zeroSensors();
        mAzimuthMotor.set(ControlMode.Position, mConstants.kAzimuthEncoderHomeOffset);
        Timer.delay(1);
        mAzimuthMotor.set(
            ControlMode.Position,
            radiansToEncoderUnits(Math.PI) + mConstants.kAzimuthEncoderHomeOffset
        );
        Timer.delay(1);
        mAzimuthMotor.set(
            ControlMode.Position,
            radiansToEncoderUnits(Math.PI / 2.0) + mConstants.kAzimuthEncoderHomeOffset
        );
        Timer.delay(1);
        mAzimuthMotor.set(
            ControlMode.Position,
            radiansToEncoderUnits(-Math.PI / 2.0) + mConstants.kAzimuthEncoderHomeOffset
        );
        Timer.delay(1);
        zeroSensors();
        return true;
        // return driveMotorPassed && azimuthMotorPassed;
    }

    @Override
    public double getAzimuthVelocity() {
        return mAzimuthMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getAzimuthPosition() {
        return mPeriodicIO.azimuth_encoder_ticks;
    }

    @Override
    public double getAzimuthPositionDemand() {
        return mPeriodicIO.azimuth_demand;
    }

    @Override
    public double getAzimuthError() {
        return mAzimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms;
    }

    @Override
    public double getDriveVelocityDemand() {
        return mPeriodicIO.drive_demand;
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveDistance() {
        return Drive.rotationsToInches(
            mPeriodicIO.drive_encoder_ticks / Drive.DRIVE_ENCODER_PPR
        );
    }

    /**
     * @param ticks azimuth ticks
     */
    public synchronized double encoderUnitsToRadians(double ticks) {
        return ticks / mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @return azimuth ticks
     */
    public synchronized double radiansToEncoderUnits(double radians) {
        return radians * mConstants.kAzimuthTicksPerRadian;
    }

    /**
     * @param ticks drive ticks
     */
    public synchronized double encoderUnitsToDistance(double ticks) {
        return ticks * mConstants.kDriveTicksPerUnitDistance;
    }

    /**
     * @return drive ticks
     */
    public synchronized double distanceToEncoderUnits(double distance) {
        return distance / mConstants.kDriveTicksPerUnitDistance;
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians((encoderUnitsToRadians(getAzimuthPosition())));
    }

    public synchronized Rotation2d getAngleToDegrees() {
        var azimuthPosition = getAzimuthPosition();
        var radians = encoderUnitsToRadians(azimuthPosition);
        var degrees = (180 / Math.PI) * radians;
        return Rotation2d.fromDegrees(degrees);
    }

    public synchronized double getRawAngle() {
        return encoderUnitsToRadians(getAzimuthPosition());
    }

    public synchronized double getUnwrappedAngleDegrees() {
        return Math.toDegrees(encoderUnitsToRadians(getAzimuthPosition()));
    }

    public synchronized double getRawLinearVelocity() {
        return mPeriodicIO.velocity_ticks_per_100ms * 10;
    }

    public synchronized double getLinearVelocity() {
        return encoderUnitsToDistance(getRawLinearVelocity());
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        mDriveMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized void setAzimuthBrakeMode(boolean brake_mode) {
        mAzimuthMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public synchronized double getDrivePercentOutput() {
        return mDriveMotor.getMotorOutputPercent();
    }

    public synchronized boolean isAzimuthAtTarget() {
        return Util.epsilonEquals(
            mPeriodicIO.azimuth_demand,
            getAzimuthPosition(),
            mConstants.kAzimuthClosedLoopAllowableError
        );
    }

    @Override
    public String toString() {
        return (
            "SwerveModule{ " +
            mConstants.kDriveMotorName +
            " id: " +
            mDriveMotor.getDeviceID() +
            "  " +
            mConstants.kAzimuthMotorName +
            " id: " +
            mAzimuthMotor.getDeviceID() +
            " offset: " +
            mConstants.kAzimuthEncoderHomeOffset +
            " invertSensor: " +
            mConstants.kInvertAzimuthSensorPhase +
            " invertAzimuth: " +
            mConstants.kInvertAzimuth +
            " encPPR: " +
            Drive.DRIVE_ENCODER_PPR +
            " }"
        );
    }
}
