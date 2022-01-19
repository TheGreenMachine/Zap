package com.team1816.lib.subsystems;

public interface DifferentialDrivetrain extends TrackableDrivetrain {
    double getLeftVelocityNativeUnits();
    double getRightVelocityNativeUnits();

    double getLeftVelocityDemand();
    double getRightVelocityDemand();

    double getLeftVelocityError();
    double getRightVelocityError();

    double getLeftEncoderDistance();
    double getRightEncoderDistance();
}
