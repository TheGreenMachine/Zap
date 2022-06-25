package com.team1816.lib.subsystems;

public interface DifferentialDrivetrain extends TrackableDrivetrain {
    double getLeftVelocityActual();
    double getRightVelocityActual();

    double getLeftVelocityDemand();
    double getRightVelocityDemand();

    double getLeftError();
    double getRightError();

    double getLeftDistance();
    double getRightDistance();
}
