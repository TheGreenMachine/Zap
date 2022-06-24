package com.team1816.season.controlboard;

import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.LogitechController;
import com.team1816.season.Constants;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

@Singleton
public class TankDemoControlBoard implements IControlBoard {

    private final Controller mController;
    private double drivetrainMultiplier = 0.25;

    public TankDemoControlBoard() {
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
    public boolean getSuperstructure() {
        return false;
    }

    @Override
    public boolean getCameraToggle() {
        return false;
    }

    @Override
    public boolean getRaiseBucket() {
        return false;
    }

    @Override
    public boolean getLowerBucket() {
        return false;
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
    public double getTurretXVal() {
        System.out.println(
            "turret field_following teleop control not implemented for TankDemoMode!"
        );
        return 0;
    }

    @Override
    public double getTurretYVal() {
        System.out.println(
            "turret field_following teleop control not implemented for TankDemoMode!"
        );
        return 0;
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
    public boolean getYeetShot() {
        return mController.getTrigger(Controller.Axis.RIGHT_TRIGGER);
    }

    @Override
    public boolean getAutoAim() {
        return false;
    }

    @Override
    public boolean getShoot() {
        return false;
    }

    @Override
    public boolean getAutoClimb() {
        return false;
    }

    @Override
    public boolean getBottomClamp() {
        return false;
    }

    @Override
    public boolean getTopClamp() {
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
    public boolean getBrakeMode() {
        return false;
    }

    @Override
    public boolean getSlowMode() {
        return false;
    }

    @Override
    public boolean getUnlockClimber() {
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
    public boolean getUseManualShoot() {
        return false;
    }

    @Override
    public boolean getZeroPose() {
        return false;
    }

    @Override
    public boolean getQuickTurnMode() {
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
}
