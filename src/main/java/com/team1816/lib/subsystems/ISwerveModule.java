package com.team1816.lib.subsystems;

public interface ISwerveModule {
    String getSubsystemName();

    //    double getAzimuthVelocity();
    double getAzimuthPosition();
    //    double getAzimuthPositionDemand();
    double getAzimuthError();
    double getAzimuthDemand();

    double getDriveVelocity();
    double getDriveVelocityDemand();
    //    double getDriveDistance();
    double getDriveError();
}
