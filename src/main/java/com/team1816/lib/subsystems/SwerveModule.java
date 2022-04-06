package com.team1816.lib.subsystems;

import static com.team1816.lib.subsystems.Drive.NAME;
import static com.team1816.lib.subsystems.Drive.factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.lib.math.DriveConversions;
import com.team1816.lib.math.SwerveKinematics;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule implements ISwerveModule {

    // Components
    private final IMotorControllerEnhanced mDriveMotor;
    private final IMotorControllerEnhanced mAzimuthMotor;
    public static CANCoder mCanCoder;
    public double mVelDemand;
    public double mAzmDemand;

    // State
    private boolean isBrakeMode = false;

    // Constants
    private final Constants.Swerve mConstants;
    private final double allowableError;

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

        mDriveMotor.configOpenloopRamp(0.25, Constants.kCANTimeoutMs);
        mAzimuthMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 18, 28, 1),
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

        allowableError = 5; // TODO this is a dummy value for checkSystem

        /* Angle Encoder Config */
        mCanCoder = canCoder;

        System.out.println("  " + this);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState desired_state = SwerveKinematics.optimize(
            desiredState,
            getState().angle
        ); // desiredState; //
        if (!isOpenLoop) {
            mVelDemand =
                DriveConversions.metersPerSecondToTicksPer100ms(
                    desired_state.speedMetersPerSecond
                );
            mDriveMotor.set(ControlMode.Velocity, mVelDemand);
        } else {
            mDriveMotor.set(
                ControlMode.PercentOutput,
                desired_state.speedMetersPerSecond
            ); // w/out conversion, would be lying to it
        }
        mAzmDemand =
            DriveConversions.convertDegreesToTicks(desired_state.angle.getDegrees()) +
            mConstants.kAzimuthEncoderHomeOffset;
        mAzimuthMotor.set(ControlMode.Position, mAzmDemand);
    }

    public SwerveModuleState getState() {
        double velocity =
            DriveConversions.convertTicksToMeters(
                mDriveMotor.getSelectedSensorVelocity(0)
            ) *
            10;
        Rotation2d angle = Rotation2d.fromDegrees(
            DriveConversions.convertTicksToDegrees(
                mAzimuthMotor.getSelectedSensorPosition(0) -
                mConstants.kAzimuthEncoderHomeOffset
            )
        );
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public String getSubsystemName() {
        return mConstants.kName;
    }

    @Override
    public double getAzimuthPosition() {
        return mAzimuthMotor.getSelectedSensorPosition(0);
    }

    @Override
    public double getAzimuthError() {
        return mAzimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getAzimuthDemand() {
        return mAzmDemand;
    }

    @Override
    public double getDriveVelocity() {
        return mDriveMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getDriveVelocityDemand() {
        return mVelDemand;
    }

    @Override
    public double getDriveError() {
        return mDriveMotor.getClosedLoopError(0);
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        //        mDriveMotor.setNeutralMode(brake_mode ? NeutralMode.Brake : NeutralMode.Coast);
        if (brake_mode) {
            mDriveMotor.set(ControlMode.Velocity, 0);
        }
        isBrakeMode = brake_mode;
    }

    public boolean checkSystem() {
        boolean checkDrive = true;
        double actualMaxTicks = factory.getConstant(NAME, "maxTicks") / 0.8; // if this isn't calculated right this test will fail
        mDriveMotor.set(ControlMode.PercentOutput, 0.2);
        Timer.delay(1);
        if (
            Math.abs(mDriveMotor.getSelectedSensorVelocity(0) - 0.2 * actualMaxTicks) >
            actualMaxTicks /
            50
        ) {
            checkDrive = false;
        }
        mDriveMotor.set(ControlMode.PercentOutput, -0.2);
        Timer.delay(1);
        if (
            Math.abs(mDriveMotor.getSelectedSensorVelocity(0) + 0.2 * actualMaxTicks) >
            actualMaxTicks /
            50
        ) {
            checkDrive = false;
        }

        boolean checkAzimuth = true;
        double setPoint = mConstants.kAzimuthEncoderHomeOffset;
        Timer.delay(1);
        for (int i = 0; i < 4; i++) {
            mAzimuthMotor.set(ControlMode.Position, setPoint);
            Timer.delay(1);
            if (
                Math.abs(mAzimuthMotor.getSelectedSensorPosition(0) - setPoint) >
                allowableError
            ) {
                checkAzimuth = false;
                break;
            }
            setPoint += DriveConversions.convertRadiansToTicks(Math.PI / 2);
        }

        return checkDrive && checkAzimuth;
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
            mConstants.kInvertAzimuth
        );
    }
}
