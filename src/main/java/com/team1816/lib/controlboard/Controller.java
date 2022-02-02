package com.team1816.lib.controlboard;

import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import java.util.EnumMap;

public abstract class Controller {

    public interface Factory {
        Controller getControllerInstance(int port);
    }

    protected final Joystick mController;
    protected final EnumMap<Button, Integer> mJoystickButtonMap = new EnumMap<>(
        Button.class
    );
    protected final EnumMap<Axis, Integer> mJoystickAxisMap = new EnumMap<>(Axis.class);

    public enum Axis {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
    }

    public enum Button {
        A,
        B,
        X,
        Y,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        BACK,
        START,
        L_JOYSTICK,
        R_JOYSTICK,
    }

    public Controller(int port) {
        mController = new Joystick(port);
    }

    public void setRumble(boolean on) {
        mController.setRumble(GenericHID.RumbleType.kRightRumble, on ? 1 : 0);
    }

    public int getDPad() {
        return mController.getPOV();
    }

    public boolean getButton(Button button) {
        if (!mJoystickButtonMap.containsKey(button)) return false;
        return mController.getRawButton(mJoystickButtonMap.get(button));
    }

    //Treats an Axis like a button
    public boolean getTrigger(Axis axis) {
        if (!mJoystickAxisMap.containsKey(axis)) return false;
        return (
            mController.getRawAxis(mJoystickAxisMap.get(axis)) >
            Constants.kJoystickThreshold
        );
    }

    public double getJoystick(Axis axis) {
        if (!mJoystickAxisMap.containsKey(axis)) return 0;
        return mController.getRawAxis(mJoystickAxisMap.get(axis));
    }
}
