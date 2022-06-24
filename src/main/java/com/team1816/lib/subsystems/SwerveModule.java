package com.team1816.lib.subsystems;

import static com.team1816.lib.subsystems.Drive.NAME;
import static com.team1816.lib.subsystems.Drive.factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.math.DriveConversions;
import com.team1816.lib.math.SwerveKinematics;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule implements ISwerveModule {

    // Components
    private final IGreenMotor driveMotor;
    private final IGreenMotor azimuthMotor; // angle motor (the pivot part of a shopping cart except motorized)
    public final CANCoder canCoder;
    public double driveDemand;
    public double azimuthDemand;

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
        driveMotor =
            factory.getMotor(
                subsystemName,
                constants.kDriveMotorName,
                factory.getSubsystem(subsystemName).swerveModules.drivePID,
                -1
            );

        /* Azimuth (Angle) Motor Config */
        azimuthMotor =
            factory.getMotor(
                subsystemName,
                constants.kAzimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID,
                canCoder.getDeviceID()
            );

        driveMotor.configOpenloopRamp(0.25, Constants.kCANTimeoutMs);
        azimuthMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 18, 28, 1),
            Constants.kLongCANTimeoutMs
        );

        azimuthMotor.configPeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        azimuthMotor.configPeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);

        azimuthMotor.setSensorPhase(constants.kInvertAzimuthSensorPhase);
        azimuthMotor.setNeutralMode(NeutralMode.Brake);

        azimuthMotor.configAllowableClosedloopError(
            0,
            constants.kAzimuthPid.allowableError,
            Constants.kLongCANTimeoutMs
        );

        allowableError = 5; // TODO this is a dummy value for checkSystem

        /* Angle Encoder Config */
        this.canCoder = canCoder;

        System.out.println("  " + this);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState desired_state = SwerveKinematics.optimize(
            desiredState,
            getState().angle
        );
        driveDemand =
            DriveConversions.metersPerSecondToTicksPer100ms(
                desired_state.speedMetersPerSecond
            );
        if (!isOpenLoop) {
            driveMotor.set(ControlMode.Velocity, driveDemand);
        } else {
            driveMotor.set(ControlMode.PercentOutput, desired_state.speedMetersPerSecond); // lying to it - speedMetersPerSecond passed in is actually percent output (1 to -1)
        }
        azimuthDemand =
            DriveConversions.convertDegreesToTicks(desired_state.angle.getDegrees()) +
            mConstants.kAzimuthEncoderHomeOffset;
        azimuthMotor.set(ControlMode.Position, azimuthDemand);
    }

    public SwerveModuleState getState() {
        double velocity =
            DriveConversions.convertTicksToMeters(
                driveMotor.getSelectedSensorVelocity(0)
            ) *
            10;
        Rotation2d angle = Rotation2d.fromDegrees(
            DriveConversions.convertTicksToDegrees(
                azimuthMotor.getSelectedSensorPosition(0) -
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
    public double getAzimuthActual() {
        return azimuthMotor.getSelectedSensorPosition(0);
    }

    @Override
    public double getAzimuthError() {
        return azimuthMotor.getClosedLoopError(0);
    }

    @Override
    public double getAzimuthDemand() {
        return azimuthDemand;
    }

    @Override
    public double getDriveActual() {
        return driveMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public double getDriveDemand() {
        return driveDemand;
    }

    @Override
    public double getDriveError() {
        return driveMotor.getClosedLoopError(0);
    }

    public synchronized void setDriveBrakeMode(boolean brake_mode) {
        if (brake_mode) {
            driveMotor.set(ControlMode.Velocity, 0);
        }
        isBrakeMode = brake_mode;
    }

    public boolean checkSystem() {
        boolean checkDrive = true;
        double actualMaxTicks = factory.getConstant(NAME, "maxTicks"); // if this isn't calculated right this test will fail
        driveMotor.set(ControlMode.PercentOutput, 0.2);
        Timer.delay(1);
        if (
            Math.abs(driveMotor.getSelectedSensorVelocity(0) - 0.2 * actualMaxTicks) >
            actualMaxTicks /
            50
        ) {
            checkDrive = false;
        }
        driveMotor.set(ControlMode.PercentOutput, -0.2);
        Timer.delay(1);
        if (
            Math.abs(driveMotor.getSelectedSensorVelocity(0) + 0.2 * actualMaxTicks) >
            actualMaxTicks /
            50
        ) {
            checkDrive = false;
        }

        boolean checkAzimuth = true;
        double setPoint = mConstants.kAzimuthEncoderHomeOffset;
        Timer.delay(1);
        for (int i = 0; i < 4; i++) {
            azimuthMotor.set(ControlMode.Position, setPoint);
            Timer.delay(1);
            if (
                Math.abs(azimuthMotor.getSelectedSensorPosition(0) - setPoint) >
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
            driveMotor.getDeviceID() +
            "  " +
            mConstants.kAzimuthMotorName +
            " id: " +
            azimuthMotor.getDeviceID() +
            " offset: " +
            mConstants.kAzimuthEncoderHomeOffset +
            " invertSensor: " +
            mConstants.kInvertAzimuthSensorPhase +
            " invertAzimuth: " +
            mConstants.kInvertAzimuth
        );
    }
}
