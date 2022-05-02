package com.team1816.lib.subsystems;

public interface ISwerveModule {
    String getSubsystemName();

    //    double getAzimuthVelocity();
    double getAzimuthActual();
    //    double getAzimuthPositionDemand();
    double getAzimuthError();
    double getAzimuthDemand();

    double getDriveActual();
    double getDriveDemand();
    //    double getDriveDistance();
    double getDriveError();
}
