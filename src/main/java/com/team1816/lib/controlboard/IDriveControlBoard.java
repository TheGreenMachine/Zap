package com.team1816.lib.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    double getStrafe();

    boolean getBrakeMode();

    boolean getSlowMode();

    boolean getUnlockClimber();

    boolean getCollectorToggle();

    boolean getCollectorBackspin();

    boolean getUseManualShoot();

    boolean getZeroPose();

    boolean getQuickTurnMode();

    int getDriverClimber();

    double getDPad(); // TODO: what is the purpose of this???

    boolean getFieldRelative();
}
