package com.team1816.lib.controlboard;

public interface IButtonControlBoard {
    void reset();

    void setRumble(boolean on);

    boolean getSuperstructure();

    boolean getFeederFlapOut();

    boolean getFeederFlapIn();

    boolean getClimberUp();

    boolean getClimberDown();

    double getTurretXVal(); // front-to-back portion of angle - using X and Y as used in the path generator! may be the wrong axes!

    double getTurretYVal(); // side-to-side portion of angle

    boolean getTurretJogLeft();

    boolean getTurretJogRight();

    boolean getRevShooter();

    boolean getAutoAim();

    boolean getShoot();

    boolean getIncrementClimberStage();

    boolean getBottomClamp();

    boolean getTopClamp();

    boolean getFieldFollowing();
}
