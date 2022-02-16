package com.team1816.season.subsystems;

import static com.team1816.lib.math.DriveConversions.metersPerSecondToTicksPer100ms;
import static com.team1816.season.subsystems.Drive.*;
import static com.team1816.season.subsystems.SwerveModule.ControlState.OPEN_LOOP;
import static com.team1816.season.subsystems.SwerveModule.ControlState.VELOCITY;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.math.DriveConversions;
import com.team1816.lib.subsystems.ISwerveModule;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule implements ISwerveModule {

    public static class PeriodicIO {

        // INPUTS
        public SwerveModuleState desired_state = new SwerveModuleState();
        public double velocity_ticks_per_100ms = 0;
        public double azimuth_actual_degrees = 0;

        // OUTPUTS
        public double drive_demand = 0;
        public Rotation2d azimuth_position = Constants.EmptyRotation; // actual desired demand in encoder units, not adjusted for home offset
    }

    public enum ControlState {
        OPEN_LOOP,
        VELOCITY,
    }

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;
    public static CANCoder mCanCoder;

    // simulation
    private double driveSimTicksPer100ms = 0, azimuthEncoderSimPosition = 0;

    // State
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ControlState mControlState = OPEN_LOOP;
    private boolean isBrakeMode = false;
    private Translation2d startingPosition; // -ginget

    // Constants
    private final Constants.Swerve mConstants;
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.Swerve.driveKS,
        Constants.Swerve.driveKV,
        Constants.Swerve.driveKA
    );

    public SwerveModule(
        String subsystemName,
        Constants.Swerve constants,
        CANCoder canCoder
    ) {

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
                factory.getSubsystem(subsystemName).swerveModules.drivePID,
                -1
            );

        /* Azimuth (Angle) Motor Config */
        mAzimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID,
                canCoder.getDeviceID()
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

        System.out.println("  " + this);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        mControlState = isOpenLoop ? OPEN_LOOP : VELOCITY;
        mPeriodicIO.desired_state = desiredState; // ModuleState.optimize(desiredState, getState().angle); // here
        if (mControlState == VELOCITY) {
            mPeriodicIO.drive_demand =
                metersPerSecondToTicksPer100ms(
                    mPeriodicIO.desired_state.speedMetersPerSecond
                ); // driveDemand now in ticks
        } else {
            mPeriodicIO.drive_demand =
                mPeriodicIO.desired_state.speedMetersPerSecond /
                Units.inchesToMeters(Constants.kPathFollowingMaxVel); // driveDemand now percent output
        }
        mPeriodicIO.azimuth_position =
            Rotation2d.fromDegrees(mPeriodicIO.desired_state.angle.getDegrees());
    }

    public SwerveModuleState getState() {
        double velocity =
            (mPeriodicIO.velocity_ticks_per_100ms * 10 / DRIVE_ENCODER_PPR) *
            Constants.kWheelCircumferenceMeters; // proper conversion?
        Rotation2d angle = Rotation2d.fromDegrees(
            mPeriodicIO.azimuth_actual_degrees -
            DriveConversions.convertTicksToDegrees(mConstants.kAzimuthEncoderHomeOffset)
        );
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public String getSubsystemName() {
        return null;
    }

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
        return metersPerSecondToTicksPer100ms(
            mPeriodicIO.desired_state.speedMetersPerSecond
        );
    }

    @Override
    public double getDriveVelocityDemand() {
        return mPeriodicIO.drive_demand;
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
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

    //
    //    /**
    //     * @param ticks drive ticks
    //     */
    //    public synchronized double encoderUnitsToDistance(double ticks) {
    //        return ticks * mConstants.kDriveTicksPerUnitDistance;
    //

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
