package com.team1816.frcSeason.subsystems;

import static com.team1816.frcSeason.subsystems.Drive.maxVelTicksPer100ms;
import static com.team1816.frcSeason.subsystems.SwerveModule.ControlState.OPEN_LOOP;
import static com.team1816.frcSeason.subsystems.SwerveModule.ControlState.VELOCITY;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.frcSeason.Constants;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.math.Conversions;
import com.team1816.lib.subsystems.ISwerveModule;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.ModuleState;
import com.team254.lib.geometry.Pose2d;
import com.team1816.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        public SwerveModuleState desired_state;

        // OUTPUTS
        public double drive_demand;
        public Rotation2d azimuth_position; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP,
        VELOCITY,
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;
    private static CANCoder mCanCoder;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = OPEN_LOOP;
    private boolean isBrakeMode = false;
    private double driveEncoderSimPosition = 0;
    private double previousEncDistance = 0;
    private Translation2d position = Translation2d.identity();
    private final Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();
    private double lastAngle;

    // Constants
    private final Constants.Swerve mConstants;
    private static final double TICK_RATIO_PER_LOOP = Constants.kLooperDt / 0.1;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);


    public SwerveModule(
        String subsystemName,
        Constants.Swerve constants,
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

        /* Drive Motor Config */
        mDriveMotor =
            factory.getMotor(
                subsystemName,
                constants.kDriveMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID
            );

        /* Azimuth (Angle) Motor Config */
        mAzimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.drivePID
            );

        mAzimuthMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 25, 0, 0),
            Constants.kLongCANTimeoutMs
        );

        mAzimuthMotor.configPeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        mAzimuthMotor.configPeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);

        mAzimuthMotor.setSensorPhase(constants.kInvertAzimuthSensorPhase);
        mAzimuthMotor.setNeutralMode(NeutralMode.Brake);

        mAzimuthMotor.configAllowableClosedloopError(
            0,
            constants.kAzimuthClosedLoopAllowableError,
            Constants.kLongCANTimeoutMs
        );

        /* Angle Encoder Config */
        mCanCoder = canCoder;

        /* Set initial Angle and Pose */
        this.startingPosition = startingPosition;
        lastAngle = getState().angle.getDegrees();

        System.out.println("  " + this);

        zeroSensors();
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

//    public synchronized void setOpenLoop(double speed, Rotation2d azimuth) {
//        if (mControlState != OPEN_LOOP) {
//            mControlState = OPEN_LOOP;
//        }
//        mPeriodicIO.drive_demand = speed;
//        mPeriodicIO.azimuth_position = azimuth;
//    }
//
//    public synchronized void setVelocity(double speed, Rotation2d azimuth) {
//        if (mControlState != ControlState.VELOCITY) {
//            mControlState = ControlState.VELOCITY;
//        }
//
//    }

    @Override
    public void readPeriodicInputs() {
        if (RobotBase.isSimulation()) {
            double adjDriveDemand = mPeriodicIO.drive_demand;
            if (mControlState == OPEN_LOOP) {
                adjDriveDemand = adjDriveDemand * maxVelTicksPer100ms;
            }
            driveEncoderSimPosition += adjDriveDemand * TICK_RATIO_PER_LOOP;
            mPeriodicIO.drive_encoder_ticks = driveEncoderSimPosition;
            mPeriodicIO.velocity_ticks_per_100ms = adjDriveDemand;
            mPeriodicIO.azimuth_encoder_ticks = mPeriodicIO.azimuth_position;
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
                normalizedEncoderTicks & Constants.Swerve.AZIMUTH_TICK_MASK;
        }
    }

//    public void setOpenLoopRampRate(double openLoopRampRate) {
//        mDriveMotor.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
//    }

    @Override
    public void writePeriodicOutputs() {
        // TODO setDesiredState is basically this call?

        if(mControlState == OPEN_LOOP) {
            double percentOutput = mPeriodicIO.desired_state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon( // TODO change to one of our conversions that actually makes sense
                mPeriodicIO.desired_state.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio
            );
            mDriveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(mPeriodicIO.desired_state.speedMetersPerSecond)
            );
        }
        double angle = (Math.abs(mPeriodicIO.desired_state.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01)) ?
            lastAngle : mPeriodicIO.desired_state.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAzimuthMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.angleGearRatio));

//        if (mControlState == OPEN_LOOP) {
//            if (
//                Util.epsilonEquals(
//                    mPeriodicIO.drive_demand,
//                    0.0,
//                    mConstants.kDriveDeadband
//                )
//            ) { // don't move if
//                // throttle is 0
//                stop();
//            } else {
//                mDriveMotor.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
//            }
//        } else if (mControlState == ControlState.VELOCITY) {
//            //            System.out.println(mConstants.kName + " drive demand: " + mPeriodicIO.drive_demand);
//            mDriveMotor.set(ControlMode.Velocity, mPeriodicIO.drive_demand);
//        }

//        double demandedPosition;
//        var upDistance = mPeriodicIO.azimuth_position - mPeriodicIO.azimuth_encoder_ticks;
//        if (mPeriodicIO.azimuth_position < mPeriodicIO.azimuth_encoder_ticks) {
//            upDistance += 4096;
//        }
//
//        var downDistance = upDistance - 4096;
//
//        if (Math.abs(upDistance) < Math.abs(downDistance)) {
//            demandedPosition = mPeriodicIO.azimuth_encoder_ticks_unmasked + upDistance;
//        } else {
//            demandedPosition = mPeriodicIO.azimuth_encoder_ticks_unmasked + downDistance;
//        }
//
//        var offsetDemand =
//            ((int) (demandedPosition + mConstants.kAzimuthEncoderHomeOffset));
//
//        mAzimuthMotor.set(ControlMode.Position, offsetDemand);
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
        // TODO there seems to be an easier way to zero sensors using setSelectedSensorPosition (see BaseFalconSwerve)
        if (mAzimuthMotor instanceof TalonSRX) {
            var sensors = ((TalonSRX) mAzimuthMotor).getSensorCollection();
            sensors.setQuadraturePosition(
                sensors.getPulseWidthPosition() & Constants.Swerve.AZIMUTH_TICK_MASK,
                Constants.kLongCANTimeoutMs
            );
        }
        mDriveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
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
        return true; // TODO actually make this method check the system
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
        return mPeriodicIO.azimuth_position;
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
        if (mControlState == OPEN_LOOP) {
            return mPeriodicIO.drive_demand * maxVelTicksPer100ms;
        }
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

    private int getAzimuthPosAbsolute() {
        if (mAzimuthMotor instanceof TalonSRX) {
            int rawValue =
                ((TalonSRX) mAzimuthMotor).getSensorCollection().getPulseWidthPosition() &
                    Constants.Swerve.AZIMUTH_TICK_MASK;
            return rawValue;
        }
        return 0;
    }

    public synchronized Rotation2d getAngle() {
        return new Rotation2d(encoderUnitsToRadians(getAzimuthPosition()));
    }

    public synchronized Rotation2d getAngleToDegrees() {
        var azimuthPosition = getAzimuthPosition();
        var radians = encoderUnitsToRadians(azimuthPosition);
        var degrees = (180 / Math.PI) * radians;
        return Rotation2d.fromDegrees(degrees);
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

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        mControlState = isOpenLoop ? OPEN_LOOP : VELOCITY;
        mPeriodicIO.desired_state = ModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        // is this adjustment necessary?
        var isFront = mConstants.kName.startsWith("front");
        var sign = isFront ? 1 : -1;
        var azimuthAdjustmentRadians =
            sign * Math.toRadians(Constants.Swerve.AZIMUTH_ADJUSTMENT_OFFSET_DEGREES);

        mPeriodicIO.drive_demand = mPeriodicIO.desired_state.speedMetersPerSecond;
        mPeriodicIO.azimuth_position = mPeriodicIO.desired_state.angle.rotateBy(new Rotation2d(azimuthAdjustmentRadians));
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        mAzimuthMotor.setSelectedSensorPosition(absolutePosition, 0, Constants.kCANTimeoutMs); // TODO see if kCANTimeoutMs or kLongCANTimeoutMs
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(mCanCoder.getAbsolutePosition());
    }


    public SwerveModuleState getState(){
        double velocity = getDriveVelocityDemand(); // Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(0), Constants.kWheelCircumferenceInches, Constants.driveGearRatio);
        Rotation2d angle = getAngle(); // Rotation2d.fromDegrees(Conversions.falconToDegrees(mAzimuthMotor.getSelectedSensorPosition(0), Constants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
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

