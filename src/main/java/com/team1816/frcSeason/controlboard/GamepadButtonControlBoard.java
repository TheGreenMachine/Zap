package com.team1816.frcSeason.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.frcSeason.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team254.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class GamepadButtonControlBoard implements IButtonControlBoard {

    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    @Inject
    private GamepadButtonControlBoard(Controller.Factory controller) {
        mController = controller.getControllerInstance(Constants.kButtonGamepadPort);
        reset();
    }

    private final Controller mController;

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public void reset() {
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    // Spinner
    @Override
    public boolean getSpinnerReset() {
        return mController.getButton(Controller.Button.START);
    }

    @Override
    public boolean getSpinnerColor() {
        return mController.getButton(Controller.Button.X);
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return mController.getButton(Controller.Button.B);
    }

    // Turret
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
        return mController.getDPad() == 180;
    }

    // Feeder Flap
    @Override
    public boolean getFeederFlapOut() {
        return mController.getButton(Controller.Button.Y);
    }

    @Override
    public boolean getFeederFlapIn() {
        return mController.getButton(Controller.Button.A);
    }

    @Override
    public boolean getClimberUp() {
        return false;
    }

    @Override
    public boolean getClimberDown() {
        return mController.getDPad() == 180;
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(Controller.Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getAutoAim() {
        return mController.getButton(Controller.Button.LEFT_BUMPER);
    }

    @Override
    public boolean getUnusedButton() {
        return mController.getButton(Controller.Button.RIGHT_BUMPER);
    }

    @Override
    public boolean getClimberDeploy() {
        return mController.getButton(Controller.Button.Y);
    }
}
