package com.team1816.lib.subsystems;

public interface ISwerveModule {
    String getName();

    // angle degrees
    double getActualAzimuth();
    double getAzimuthError();
    double getDesiredAzimuth();

    // velocity ticks/100ms
    double getActualDrive();
    double getDesiredDrive();
    double getDriveError();
}
