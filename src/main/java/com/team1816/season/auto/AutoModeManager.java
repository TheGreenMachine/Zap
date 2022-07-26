package com.team1816.season.auto;

import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

@Singleton
public class AutoModeManager {

    // Auto mode selection
    private final SendableChooser<DesiredAuto> autoModeChooser;
    private DesiredAuto desiredAuto;

    // Auto mode execution
    private AutoMode autoMode;
    private static Thread autoModeThread;

    public AutoModeManager() {
        // Sendable chooser represents the dropdown menu in shuffleboard to pick our desired auto mode
        autoModeChooser = new SendableChooser<>();
        // Populate dropdown menu with all possible auto modes (represented as DesiredMode enums)
        SmartDashboard.putData("Auto mode", autoModeChooser);

        for (DesiredAuto desiredAuto : DesiredAuto.values()) {
            autoModeChooser.addOption(desiredAuto.name(), desiredAuto);
        }
        autoModeChooser.setDefaultOption(
            DesiredAuto.DRIVE_STRAIGHT.name(),
            DesiredAuto.DRIVE_STRAIGHT
        );

        // Initialize auto mode and its respective thread
        reset();
    }

    public void reset() {
        autoMode = new DriveStraightMode();
        autoModeThread = new Thread(autoMode::run);
        desiredAuto = DesiredAuto.DRIVE_STRAIGHT;
    }

    public boolean update() {
        DesiredAuto selectedAuto = autoModeChooser.getSelected();
        boolean autoChanged = desiredAuto != selectedAuto;

        // if auto has been changed, update selected auto mode + thread
        if (autoChanged) {
            System.out.println(
                "Auto changed from: " + desiredAuto + ", to: " + selectedAuto.name()
            );

            autoMode = generateAutoMode(selectedAuto);
            autoModeThread = new Thread(autoMode::run);
        }
        desiredAuto = selectedAuto;

        return autoChanged;
    }

    public void outputToSmartDashboard() {
        if (desiredAuto != null) {
            SmartDashboard.putString("AutoModeSelected", desiredAuto.name());
        }
    }

    public AutoMode getSelectedAuto() {
        return autoMode;
    }

    // Auto Mode Executor
    public void startAuto() {
        autoModeThread.start();
    }

    public void stopAuto() {
        if (autoMode != null) {
            autoMode.stop();
            autoModeThread = new Thread(autoMode::run);
        }
    }

    enum DesiredAuto {
        // 2020
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        LIVING_ROOM,
        DRIVE_STRAIGHT,

        // 2022
        DRIVE_STRAIGHT_SHOOT,
        TWO_BALL_A,
        TWO_BALL_B,
        FOUR_BALL_B,
        TWO_BALL_C,
        FOUR_BALL_C,
        FIVE_BALL,
        ONE_BALL_A_B,
        ONE_BALL_C_BORDER,
    }

    private AutoMode generateAutoMode(DesiredAuto mode) {
        switch (mode) {
            case DRIVE_STRAIGHT_SHOOT:
                return new DriveStraightShootMode();
            case DO_NOTHING:
                return new DoNothingMode();
            case TUNE_DRIVETRAIN:
                return new TuneDrivetrainMode();
            case LIVING_ROOM:
                return (new LivingRoomMode());
            case TWO_BALL_A:
                return (new TwoBallModeA());
            case TWO_BALL_B:
                return (new TwoBallModeB());
            case FOUR_BALL_B:
                return (new FourBallModeB());
            case TWO_BALL_C:
                return (new TwoBallModeC());
            case FOUR_BALL_C:
                return (new FourBallModeC());
            case FIVE_BALL:
                return (new FiveBallMode());
            case ONE_BALL_A_B:
                return (new OneBallA_BMode());
            case ONE_BALL_C_BORDER:
                return (new OneBallC_BorderMode());
            default:
                System.out.println("Defaulting to drive straight mode");
                return new DriveStraightMode();
        }
    }
}
