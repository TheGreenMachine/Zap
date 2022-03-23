package com.team1816.lib.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    double getStrafe();

    boolean getQuickTurn();

    boolean getSlowMode();

    boolean getUnlockClimber();

    boolean getCollectorToggle();

    boolean getCollectorBackspin();

    boolean getFeederToTrenchSpline();

    boolean getZeroPose();

    boolean getBrakeMode();

    int getDriverClimber();

    double getDPad();

    boolean getFieldRelative();

}
