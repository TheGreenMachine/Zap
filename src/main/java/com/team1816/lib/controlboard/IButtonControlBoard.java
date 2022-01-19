package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    boolean getSpinnerReset();

    boolean getFeederFlapOut();

    boolean getFeederFlapIn();

    boolean getClimberUp();

    boolean getClimberDown();

    boolean getSpinnerColor();

    boolean getSpinnerThreeTimes();

    boolean getTurretJogLeft();

    boolean getTurretJogRight();

    boolean getShoot();

    boolean getAutoAim();

    boolean getUnusedButton();

    boolean getClimberDeploy();

    boolean getFieldFollowing();
}
