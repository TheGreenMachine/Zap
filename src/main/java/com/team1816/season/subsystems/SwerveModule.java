package com.team1816.season.subsystems;

import static com.team1816.season.subsystems.Drive.metersPerSecondToTicksPer100ms;
import static com.team1816.season.subsystems.SwerveModule.ControlState.OPEN_LOOP;
import static com.team1816.season.subsystems.SwerveModule.ControlState.VELOCITY;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.season.Constants;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.ISwerveModule;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.ModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;


public class SwerveModule extends Subsystem implements ISwerveModule {

    public static class PeriodicIO {

        // INPUTS
//        public double velocity_ticks_per_100ms; // -ginget
        public SwerveModuleState desired_state = new SwerveModuleState();

        // OUTPUTS
        public double drive_demand = 0;
        public Rotation2d azimuth_position = Constants.emptyRotation; // actual desired demand in encoder units, not adjusted for home offset
        public Rotation2d previous_azimuth_position = Constants.emptyRotation;
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
    private Translation2d startingPosition; // -ginget

    // Constants
    private final Constants.Swerve mConstants;
    private static final double TICK_RATIO_PER_LOOP = Constants.kLooperDt / 0.1;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(
        String subsystemName,
        Constants.Swerve constants,
        CANCoder canCoder
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
//        this.startingPosition = startingPosition; -ginget
//        mPeriodicIO.azimuth_position = getState().angle;

        System.out.println("  " + this);

        zeroSensors();
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
        Rotation2d normalizedAngle = getAngleToDegrees();
        return normalizedAngle.rotateBy(robotHeading);
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
//            if (mControlState == OPEN_LOOP) {
//                throw new UnsupportedOperationException("swerve in open loop not supported!");
//            }
//            mPeriodicIO.velocity_ticks_per_100ms = mPeriodicIO.drive_demand;// -ginget
        } else {
//            mPeriodicIO.velocity_ticks_per_100ms = // - ginget
//                mDriveMotor.getSelectedSensorVelocity(0);

        }
    }

//    public void setOpenLoopRampRate(double openLoopRampRate) {
//        mDriveMotor.configOpenloopRamp(openLoopRampRate, Constants.kCANTimeoutMs);
//    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState != OPEN_LOOP) { // negation is safer - if an exception occurs drive with be the joystick and won't go out of control
            double velocity = metersPerSecondToTicksPer100ms(mPeriodicIO.drive_demand);
            mDriveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                feedforward.calculate(mPeriodicIO.drive_demand)
            );
        } else {
            double percentOutput = mPeriodicIO.drive_demand / Units.inchesToMeters(Constants.kPathFollowingMaxVel);
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }

        double angle = (Math.abs(mPeriodicIO.drive_demand) <= (Units.inchesToMeters(Constants.kPathFollowingMaxVel) * 0.01)) ?
            mPeriodicIO.previous_azimuth_position.getDegrees() : mPeriodicIO.desired_state.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAzimuthMotor.set(ControlMode.Position, radiansToEncoderUnits(Units.degreesToRadians(angle)));
        mPeriodicIO.azimuth_position = new Rotation2d(Units.degreesToRadians(angle));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        mControlState = isOpenLoop ? OPEN_LOOP : VELOCITY;
        mPeriodicIO.desired_state = ModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

//        // is this adjustment necessary?
//        var isFront = mConstants.kName.startsWith("front");
//        var sign = isFront ? 1 : -1;
//        var azimuthAdjustmentRadians =
//            sign * Math.toRadians(mConstants.kAzimuthAdjustmentOffset);

        mPeriodicIO.drive_demand = mPeriodicIO.desired_state.speedMetersPerSecond;
        mPeriodicIO.previous_azimuth_position = mPeriodicIO.azimuth_position;
        mPeriodicIO.azimuth_position = mPeriodicIO.desired_state.angle;
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

//    @Override
//    public double getAzimuthVelocity() {
//        return mAzimuthMotor.getSelectedSensorVelocity(0);
//    }

    @Override
    public double getAzimuthPosition() {
        return mPeriodicIO.azimuth_position.getDegrees();
    }

//    @Override
//    public double getAzimuthPositionDemand() {
//        return mPeriodicIO.azimuth_position;
//    }

    @Override
    public double getAzimuthError() {
        return mAzimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getDriveVelocity() {
        return mPeriodicIO.desired_state.speedMetersPerSecond;
    }

    @Override
    public double getDriveVelocityDemand() {
        return mPeriodicIO.drive_demand;
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
    }

//    @Override
//    public double getDriveDistance() {
//        return Drive.rotationsToInches(
//            mPeriodicIO.drive_encoder_ticks / Drive.DRIVE_ENCODER_PPR
//        );
//    }

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
//
//    /**
//     * @param ticks drive ticks
//     */
//    public synchronized double encoderUnitsToDistance(double ticks) {
//        return ticks * mConstants.kDriveTicksPerUnitDistance;
//

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

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        mDriveMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
        isBrakeMode = brake_mode;
    }

//    private void resetToAbsolute(){
//        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - mConstants.kAzimuthAdjustmentOffset , Constants.Swerve.angleGearRatio);
//        mAzimuthMotor.setSelectedSensorPosition(absolutePosition, 0, Constants.kCANTimeoutMs); // TODO see if kCANTimeoutMs or kLongCANTimeoutMs
//    }

    public Rotation2d getCanCoderHeading() { // do we need this?
        return Rotation2d.fromDegrees(mCanCoder.getAbsolutePosition());
    }


    public SwerveModuleState getState() {
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

