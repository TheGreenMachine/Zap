package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.controlboard.XboxController;
import com.team1816.season.Constants;

/*
    driver controller (xbox, logitech, or keyboard) -
    what method names (ie getStrafe, getBrakeMode) correspond to what button / trigger / joystick values
 */
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
        return mController.getTrigger(Controller.Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getUnlockClimber() {
        return mController.getButton(Controller.Button.Y);
    }

    @Override
    public boolean getBrakeMode() {
        return mController.getTrigger(Controller.Axis.LEFT_TRIGGER);
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
    public boolean getUseManualShoot() {
        return mController.getButton(Controller.Button.X);
    }

    @Override
    public boolean getZeroPose() {
        return mController.getButton(Controller.Button.START);
    }

    @Override
    public boolean getQuickTurnMode() {
        return mController.getTrigger(Controller.Axis.LEFT_TRIGGER);
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
