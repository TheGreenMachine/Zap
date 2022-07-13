package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.ControlBoardBridge;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IOperatorControlBoard;
import com.team1816.season.Constants;

/*
    operator controller (xbox, logitech, or keyboard) -
    what method names (ie setRumble, getShoot) correspond to what button / trigger / joystick values
 */
@Singleton
public class GamepadOperatorControlBoard implements IOperatorControlBoard {

    private static ControlBoardBridge controlBoardBridge = ControlBoardBridge.getInstance();

    private final Controller mController;

    @Inject
    private GamepadOperatorControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kOperatorGamepadPort);
        reset();
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public void reset() {}

    @Override
    public boolean getSuperstructure() {
        var name = "getSuperstructure"; // yaml name
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.B)
        ); // default
    }

    // Turret teleop control - note that X VAL doesn't necessarily correspond to joystick X AXIS
    // TODO: might want to add inverted property to axis config
    @Override
    public double getTurretXVal() {
        var name = "getTurretXVal"; // yaml name
        return getDoubleFromControllerYaml(
            name,
            mController.getJoystick(Controller.Axis.LEFT_X)
        );
    }

    @Override
    public double getTurretYVal() {
        var name = "getTurretYVal";
        return getDoubleFromControllerYaml(
            name,
            mController.getJoystick(Controller.Axis.LEFT_Y)
        );
    }

    // Turret manual teleop control - use if turret position control not working
    @Override
    public boolean getTurretJogLeft() {
        var name = "getTurretJogLeft";
        return getBooleanFromControllerYaml(name, mController.getDPad() == 270);
    }

    @Override
    public boolean getTurretJogRight() {
        var name = "getTurretJogRight";
        return getBooleanFromControllerYaml(name, mController.getDPad() == 90);
    }

    @Override
    public boolean getFieldFollowing() {
        var name = "getFieldFollowing";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.A)
        );
    }

    @Override
    public boolean getCameraToggle() {
        var name = "getCameraToggle";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.X)
        );
    }

    @Override
    public boolean getRaiseBucket() {
        var name = "getRaiseBucket";
        return getBooleanFromControllerYaml(name, mController.getDPad() == 0);
    }

    @Override
    public boolean getLowerBucket() {
        var name = "getLowerBucket";
        return getBooleanFromControllerYaml(name, mController.getDPad() == 180);
    }

    @Override
    public boolean getIncrementCamDeviation() {
        var name = "getIncrementCamDeviation";
        return getBooleanFromControllerYaml(name);
    }

    @Override
    public boolean getDecrementCamDeviation() {
        var name = "getDecrementCamDeviation";
        return getBooleanFromControllerYaml(name);
    }

    @Override
    public boolean getHood() {
        var name = "getHood";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.RIGHT_BUMPER)
        );
    }

    @Override
    public boolean getClimberUp() {
        var name = "getClimberUpDown";
        return (
            getDoubleFromControllerYaml(
                name,
                mController.getJoystick(Controller.Axis.RIGHT_Y)
            ) >
            0.5
        );
    }

    @Override
    public boolean getClimberDown() {
        var name = "getClimberUpDown";
        return (
            getDoubleFromControllerYaml(
                name,
                mController.getJoystick(Controller.Axis.RIGHT_Y)
            ) <
            -0.5
        );
    }

    @Override
    public boolean getYeetShot() {
        var name = "getYeetShot";
        return getBooleanFromControllerYaml(
            name,
            mController.getTrigger(Controller.Axis.LEFT_TRIGGER)
        );
    }

    @Override
    public boolean getAutoClimb() {
        var name = "getAutoClimb";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.Y)
        );
    }

    @Override
    public boolean getBottomClamp() {
        var name = "getBottomClamp";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.BACK)
        );
    }

    @Override
    public boolean getTopClamp() {
        var name = "getTopClamp";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.START)
        );
    }

    @Override
    public boolean getAutoAim() {
        var name = "getAutoAim";
        return getBooleanFromControllerYaml(
            name,
            mController.getButton(Controller.Button.LEFT_BUMPER)
        );
    }

    @Override
    public boolean getShoot() { // This Shoot Method Fires and Revs currently.
        var name = "getShoot";
        return getBooleanFromControllerYaml(
            name,
            mController.getTrigger(Controller.Axis.RIGHT_TRIGGER)
        );
    }

    public double getDoubleFromControllerYaml(String name) {
        return getDoubleFromControllerYaml(name, 0);
    }

    public boolean getBooleanFromControllerYaml(String name) {
        return getBooleanFromControllerYaml(name, false);
    }

    public double getDoubleFromControllerYaml(String name, double defaultVal) {
        if (controlBoardBridge != null) if (
            controlBoardBridge.getDriverAxisMap().containsKey(name)
        ) {
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
