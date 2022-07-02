package com.team1816.season.auto;

import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

@Singleton
public class AutoModeSelector {

    enum DesiredMode {
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

    private final SendableChooser<DesiredMode> autoModeChooser;

    private DesiredMode lastDesiredMode;

    private AutoMode autoMode;

    public AutoModeSelector() {
        // Sendable chooser represents the dropdown menu in shuffleboard to pick our desired auto mode
        autoModeChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto mode", autoModeChooser);

        for (DesiredMode desiredMode : DesiredMode.values()) {
            autoModeChooser.addOption(desiredMode.name(), desiredMode);
        }
        autoModeChooser.setDefaultOption(
            DesiredMode.DRIVE_STRAIGHT.name(),
            DesiredMode.DRIVE_STRAIGHT
        );

        reset();
    }

    public void update() {
        DesiredMode desiredMode = autoModeChooser.getSelected();
        if (lastDesiredMode != desiredMode) {
            System.out.println(
                "Auto selection changed, updating creator: desiredMode -> " +
                desiredMode.name()
            );
            autoMode = generateAutoMode(desiredMode);
        }
        lastDesiredMode = desiredMode;
    }

    private AutoMode generateAutoMode(DesiredMode mode) {
        switch (mode) {
            // 2020
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

    public void reset() {
        autoMode = new DriveStraightMode();
        lastDesiredMode = DesiredMode.DRIVE_STRAIGHT;
    }

    public void outputToSmartDashboard() {
        if (lastDesiredMode != null) {
            SmartDashboard.putString("AutoModeSelected", lastDesiredMode.name());
        }
    }

    public AutoMode getAutoMode() {
        return autoMode;
    }
}
