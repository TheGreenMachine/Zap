package com.team1816.lib.subsystems;

public interface TrackableDrivetrain {
    double getFieldXDistance();
    double getFieldYDistance();

    double getFieldDesiredXDistance();
    double getFieldDesiredYDistance();

    double getActualHeadingDegrees();
    double getDesiredHeadingDegrees();
}
