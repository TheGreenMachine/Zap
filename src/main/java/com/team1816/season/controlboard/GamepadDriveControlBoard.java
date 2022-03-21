package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team1816.lib.controlboard.XboxController;
import com.team1816.season.Constants;

@Singleton
public class GamepadDriveControlBoard implements IDriveControlBoard {

    private final Controller mController;

    @Inject
    private GamepadDriveControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kDriveGamepadPort);
    }

    @Override
    public double getStrafe() {
        return mController.getJoystick(Controller.Axis.LEFT_X);
    }

    @Override
    public double getThrottle() {
        return -mController.getJoystick(Controller.Axis.LEFT_Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(Controller.Axis.RIGHT_X);
    }

    @Override
    public boolean getSlowMode() {
        return mController.getButton(Controller.Button.R_JOYSTICK);
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return mController.getButton(Controller.Button.Y);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getButton(Controller.Button.R_JOYSTICK);
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(Controller.Button.LEFT_BUMPER);
    }

    @Override
    public boolean getCollectorBackspin() {
        return mController.getButton(Controller.Button.RIGHT_BUMPER);
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return mController.getButton(Controller.Button.X);
    }

    @Override
    public boolean getZeroPose() {
        return mController.getButton(Controller.Button.START);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getButton(Controller.Button.A);
    }

    @Override
    public boolean getFieldRelative() {
        return !mController.getButton(XboxController.Button.LEFT_BUMPER);
    }

    @Override
    public double getDPad() {
        return -1;
    }

    @Override
    public int getDriverClimber() {
        return 0;
    }
}
