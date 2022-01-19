package com.team1816.lib.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    double getStrafe();

    boolean getQuickTurn();

    boolean getSlowMode();

    boolean getDrivetrainFlipped();

    boolean getCollectorToggle();

    boolean getCollectorBackspin();

    boolean getFeederToTrenchSpline();

    boolean getTrenchToFeederSpline();

    boolean getBrakeMode();

    int getDriverClimber();

    double getDPad();

    boolean getFieldRelative();

    boolean getHood();
}
