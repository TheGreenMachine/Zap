package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.ControlBoardBridge;
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

    private static ControlBoardBridge controlBoardBridge = ControlBoardBridge.getInstance();

    private final Controller mController;

    @Inject
    private GamepadDriveControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kDriveGamepadPort);
    }

    @Override
    public double getStrafe() {
        var name = "getStrafe";
        return getDoubleFromControllerYaml(
            name,
            mController.getJoystick(Controller.Axis.LEFT_X)
        );
    }

    @Override
    public double getThrottle() {
        var name = "getThrottle";
        return getDoubleFromControllerYaml(
            name,
            -mController.getJoystick(Controller.Axis.LEFT_Y)
        ); //TODO: why is this still negative???
    }

    @Override
    public double getTurn() {
        var name = "getTurn";
        return getDoubleFromControllerYaml(
            name,
            mController.getJoystick(Controller.Axis.RIGHT_X)
        );
    }

    @Override
    public boolean getSlowMode() {
        var name = "getSlowMode";
        return getBooleanFromControllerYaml(
            name,
            mController.getTrigger(Controller.Axis.RIGHT_TRIGGER)
        );
    }

    @Override
    public boolean getUnlockClimber() {
        var name = "getUnlockClimber";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.Y)
        );
    }

    @Override
    public boolean getBrakeMode() {
        var name = "getBrakeMode";
        return getBooleanFromControllerYaml(
            name,
            mController.getTrigger(Controller.Axis.LEFT_TRIGGER)
        );
    }

    @Override
    public boolean getCollectorToggle() {
        var name = "getCollectorToggle";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.LEFT_BUMPER)
        );
    }

    @Override
    public boolean getCollectorBackspin() {
        var name = "getCollectorBackspin";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.RIGHT_BUMPER)
        );
    }

    @Override
    public boolean getUseManualShoot() {
        var name = "getUseManualShoot";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.X)
        );
    }

    @Override
    public boolean getZeroPose() {
        var name = "getZeroPose";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.START)
        );
    }

    @Override
    public boolean getQuickTurnMode() {
        var name = "getBrakeMode";
        return getBooleanFromControllerYaml(
            name,
            mController.getTrigger(Controller.Axis.LEFT_TRIGGER)
        );
    }

    @Override
    public boolean getFieldRelative() {
        var name = "getFieldRelative";
        return getBooleanFromControllerYaml(
            name,
            !mController.getButton(XboxController.Button.LEFT_BUMPER)
        );
    }

    @Override
    public double getDPad() { //TODO: futile
        return -1;
    }

    @Override
    public int getDriverClimber() { //TODO: futile
        return 0;
    }

    public double getDoubleFromControllerYaml(String name) {
        return getDoubleFromControllerYaml(name, 0);
    }

    public boolean getBooleanFromControllerYaml(String name) {
        return getBooleanFromControllerYaml(name, false);
    }

    public double getDoubleFromControllerYaml(String name, double defaultVal) {
        if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
            return mController.getJoystick(
                controlBoardBridge.getDriverAxisMap().get(name)
            );
        }
        if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
            return mController.getButton(
                    controlBoardBridge.getOperatorButtonMap().get(name)
                )
                ? 1
                : 0;
        }
        if (controlBoardBridge.getDriverDpadMap().containsKey(name)) {
            return (
                    mController.getDPad() ==
                    controlBoardBridge.getOperatorDpadMap().get(name)
                )
                ? 1
                : 0;
        }

        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
            return mController.getTrigger(
                controlBoardBridge.getDriverAxisMap().get(name)
            );
        }
        if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
            return mController.getButton(
                controlBoardBridge.getDriverButtonMap().get(name)
            );
        }
        if (controlBoardBridge.getDriverDpadMap().containsKey(name)) {
            return (
                mController.getDPad() == controlBoardBridge.getDriverDpadMap().get(name)
            );
        }

        return defaultVal;
    }
}
