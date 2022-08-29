package com.team1816.lib.subsystems.drive;

public interface ISwerveModule {
    String getModuleName();

    // angle degrees
    double getActualAzimuth();
    double getAzimuthError();
    double getDesiredAzimuth();

    // velocity ticks/100ms
    double getActualDrive();
    double getDesiredDrive();
    double getDriveError();

    // Temperature C
    double getMotorTemp();
}
