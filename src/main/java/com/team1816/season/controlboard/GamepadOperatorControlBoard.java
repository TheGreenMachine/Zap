package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IOperatorControlBoard;
import com.team1816.season.Constants;

/*
    operator controller (xbox, logitech, or keyboard) -
    what method names (ie setRumble, getShoot) correspond to what button / trigger / joystick values
 */
@Singleton
public class GamepadOperatorControlBoard implements IOperatorControlBoard {

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
        return mController.getButton(Controller.Button.B);
    }

    // Turret teleop control - note that X VAL doesn't necessarily correspond to joystick X AXIS
    @Override
    public double getTurretXVal() {
        return -mController.getJoystick(Controller.Axis.LEFT_Y);
    }

    @Override
    public double getTurretYVal() {
        return -mController.getJoystick(Controller.Axis.LEFT_X);
    }

    // Turret manual teleop control - use if turret position control not working
    @Override
    public boolean getTurretJogLeft() {
        return mController.getDPad() == 270;
    }

    @Override
    public boolean getTurretJogRight() {
        return mController.getDPad() == 90;
    }

    @Override
    public boolean getFieldFollowing() {
        return mController.getButton(Controller.Button.A);
    }

    @Override
    public boolean getCameraToggle() {
        return mController.getButton(Controller.Button.X);
    }

    @Override
    public boolean getRaiseBucket() {
        return mController.getDPad() == 0;
    }

    @Override
    public boolean getLowerBucket() {
        return (mController.getDPad() == 180);
    }

    @Override
    public boolean getIncrementCamDeviation() {
        return false;
    }

    @Override
    public boolean getDecrementCamDeviation() {
        return false;
    }

    @Override
    public boolean getHood() {
        return mController.getButton(Controller.Button.RIGHT_BUMPER);
    }

    @Override
    public boolean getClimberUp() {
        return mController.getJoystick(Controller.Axis.RIGHT_Y) > 0.5;
    }

    @Override
    public boolean getClimberDown() {
        return mController.getJoystick(Controller.Axis.RIGHT_Y) < -0.5;
    }

    @Override
    public boolean getYeetShot() {
        return mController.getTrigger(Controller.Axis.LEFT_TRIGGER);
    }

    @Override
    public boolean getAutoClimb() {
        return mController.getButton(Controller.Button.Y);
    }

    @Override
    public boolean getBottomClamp() {
        return mController.getButton(Controller.Button.BACK);
    }

    @Override
    public boolean getTopClamp() {
        return mController.getButton(Controller.Button.START);
    }

    @Override
    public boolean getAutoAim() {
        return mController.getButton(Controller.Button.LEFT_BUMPER);
    }

    @Override
    public boolean getShoot() { // This Shoot Method Fires and Revs currently.
        return mController.getTrigger(Controller.Axis.RIGHT_TRIGGER);
    }
}
