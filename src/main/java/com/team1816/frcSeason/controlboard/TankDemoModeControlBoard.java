package com.team1816.frcSeason.controlboard;

import com.team1816.frcSeason.Constants;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Singleton;

@Singleton
public class TankDemoModeControlBoard implements IControlBoard {

    private static TankDemoModeControlBoard INSTANCE = null;

    private final Controller mController;
    private double drivetrainMultiplier = 0.25;

    public TankDemoModeControlBoard() {
        mController = new LogitechController(Constants.kDriveGamepadPort);
        SendableChooser<Double> speedChooser = new SendableChooser<>();
        speedChooser.setDefaultOption("Comfort", 0.25);
        speedChooser.addOption("Sport", 0.5);
        speedChooser.addOption("Plaid", 1.0);
        speedChooser.addOption("Park", 0.0);

        SmartDashboard.putData("DemoModeDriveSpeed", speedChooser);
        NetworkTableInstance
            .getDefault()
            .getTable("SmartDashboard")
            .getSubTable("DemoModeDriveSpeed")
            .addEntryListener(
                "selected",
                (table, key, entry, value, flags) -> {
                    switch (value.getString()) {
                        case "Comfort":
                            drivetrainMultiplier = 0.25;
                            break;
                        case "Sport":
                            drivetrainMultiplier = 0.5;
                            break;
                        case "Plaid":
                            drivetrainMultiplier = 1.0;
                            break;
                        default:
                            drivetrainMultiplier = 0;
                    }
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );

        drivetrainMultiplier = speedChooser.getSelected();
    }

    @Override
    public void reset() {}

    @Override
    public void setRumble(boolean on) {}

    @Override
    public boolean getSpinnerReset() {
        return false;
    }

    @Override
    public boolean getFeederFlapOut() {
        return false;
    }

    @Override
    public boolean getFeederFlapIn() {
        return false;
    }

    @Override
    public boolean getClimberUp() {
        return false;
    }

    @Override
    public boolean getClimberDown() {
        return false;
    }

    @Override
    public boolean getSpinnerColor() {
        return false;
    }

    @Override
    public boolean getSpinnerThreeTimes() {
        return false;
    }

    @Override
    public boolean getTurretJogLeft() {
        return mController.getDPad() == 270;
    }

    @Override
    public boolean getTurretJogRight() {
        return mController.getDPad() == 90;
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(Controller.Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getAutoAim() {
        return false;
    }

    @Override
    public boolean getUnusedButton() {
        return false;
    }

    @Override
    public boolean getClimberDeploy() {
        return false;
    }

    @Override
    public boolean getFieldFollowing() {
        return false;
    }

    @Override
    public double getThrottle() {
        return drivetrainMultiplier * mController.getJoystick(Controller.Axis.LEFT_Y);
    }

    @Override
    public double getTurn() {
        return drivetrainMultiplier * mController.getJoystick(Controller.Axis.RIGHT_X);
    }

    @Override
    public double getStrafe() {
        return 0;
    }

    @Override
    public boolean getQuickTurn() {
        return false;
    }

    @Override
    public boolean getSlowMode() {
        return false;
    }

    @Override
    public boolean getDrivetrainFlipped() {
        return false;
    }

    @Override
    public boolean getCollectorToggle() {
        return mController.getButton(LogitechController.Button.LEFT_BUMPER);
    }

    @Override
    public boolean getCollectorBackspin() {
        return false;
    }

    @Override
    public boolean getFeederToTrenchSpline() {
        return false;
    }

    @Override
    public boolean getTrenchToFeederSpline() {
        return false;
    }

    @Override
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public int getDriverClimber() {
        return 0;
    }

    @Override
    public double getDPad() {
        return 0;
    }

    @Override
    public boolean getFieldRelative() {
        return false;
    }

    @Override
    public boolean getHood() {
        return false;
    }
}
