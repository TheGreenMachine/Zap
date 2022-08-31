package com.team1816.lib.subsystems.drive;

public interface DifferentialDrivetrain extends TrackableDrivetrain {
    double getLeftVelocityTicksActual();
    double getRightVelocityTicksActual();

    double getLeftVelocityTicksDemand();
    double getRightVelocityTicksDemand();

    double getLeftError();
    double getRightError();

    double getLeftDistance();
    double getRightDistance();
}
