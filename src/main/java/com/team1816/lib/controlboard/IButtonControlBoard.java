package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    boolean getHopper();

    boolean getFeederFlapOut();

    boolean getFeederFlapIn();

    boolean getClimberUp();

    boolean getClimberDown();

    boolean getSpinnerColor();

    boolean getSpinnerThreeTimes();

    boolean getTurretJogLeft();

    boolean getTurretJogRight();

    boolean getRevShooter();

    boolean getAutoAim();

    boolean getShoot();

    boolean getFieldFollowing();
}
