package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;

public class ControlBoard implements IControlBoard {

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    @Inject
    private ControlBoard(
        IDriveControlBoard driveControlBoard,
        IButtonControlBoard buttonControlBoard
    ) {
        mDriveControlBoard = driveControlBoard;
        mButtonControlBoard = buttonControlBoard;
    }

    @Override
    public void reset() {}

    // Drive Control Board
    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public double getStrafe() {
        return -mDriveControlBoard.getStrafe();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getSlowMode() {
        return mDriveControlBoard.getSlowMode();
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mDriveControlBoard.getDrivetrainFlipped();
    }

    @Override
    public boolean getCollectorToggle() {
        return mDriveControlBoard.getCollectorToggle();
    }

    @Override
    public boolean getCollectorBackspin() {
        return mDriveControlBoard.getCollectorBackspin();
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mDriveControlBoard.getFeederToTrenchSpline();
    }

    @Override
    public boolean getZeroPose() {
        return mDriveControlBoard.getZeroPose();
    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public int getDriverClimber() {
        return mDriveControlBoard.getDriverClimber();
    }

    // Button Control Board
    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getTurretJogLeft() {
        return mButtonControlBoard.getTurretJogLeft();
    }

    @Override
    public boolean getTurretJogRight() {
        return mButtonControlBoard.getTurretJogRight();
    }

    @Override
    public boolean getFieldFollowing() {
        return mButtonControlBoard.getFieldFollowing();
    }

    @Override
    public boolean getAutoAim() {
        return mButtonControlBoard.getAutoAim();
    }

    @Override
    public boolean getRevShooter() {
        return mButtonControlBoard.getRevShooter();
    }

    @Override
    public boolean getSuperstructure() {
        return mButtonControlBoard.getSuperstructure();
    }

    @Override
    public boolean getFeederFlapOut() {
        return mButtonControlBoard.getFeederFlapOut();
    }

    @Override
    public boolean getFeederFlapIn() {
        return mButtonControlBoard.getFeederFlapIn();
    }

    @Override
    public boolean getClimberUp() {
        return mButtonControlBoard.getClimberUp();
    }

    @Override
    public boolean getClimberDown() {
        return mButtonControlBoard.getClimberDown();
    }

    @Override
    public double getTurretXVal() {
        return mButtonControlBoard.getTurretXVal();
    }

    @Override
    public double getTurretYVal() {
        return mButtonControlBoard.getTurretYVal();
    }

    @Override
    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    @Override
    public boolean getLowShoot() {
        return mButtonControlBoard.getLowShoot();
    }

    @Override
    public boolean getMidShoot() {
        return mButtonControlBoard.getMidShoot();
    }

    @Override
    public boolean getFarShoot() {
        return mButtonControlBoard.getFarShoot();
    }

    @Override
    public double getDPad() {
        return mDriveControlBoard.getDPad();
    }

    @Override
    public boolean getFieldRelative() {
        return mDriveControlBoard.getFieldRelative();
    }

    @Override
    public boolean getHood() {
        return mDriveControlBoard.getHood();
    }
}
