package com.team1816.season.auto;

import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import javax.inject.Singleton;

@Singleton
public class AutoModeSelector {

    private boolean hardwareFailure = false;

    enum StartingPosition {
        LEFT_HAB_2,
        RIGHT_HAB_2,
        CENTER_HAB_1,
        LEFT_HAB_1,
        RIGHT_HAB_1,
    }

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

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();

        mStartPositionChooser.setDefaultOption(
            "Center HAB 1",
            StartingPosition.CENTER_HAB_1
        );

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto mode", mModeChooser);

        // Debugging / Tuning
        mModeChooser.addOption(
            "Tune Drivetrain - FOR TESTING",
            DesiredMode.TUNE_DRIVETRAIN
        );
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Living Room - FOR TESTING", DesiredMode.LIVING_ROOM);

        // Safety Nets
        mModeChooser.setDefaultOption("Drive Straight", DesiredMode.DRIVE_STRAIGHT);
        mModeChooser.addOption("Drive Straight Shoot", DesiredMode.DRIVE_STRAIGHT_SHOOT);

        // General autos
        mModeChooser.addOption("Two Ball A", DesiredMode.TWO_BALL_A);
        mModeChooser.addOption("Two Ball B", DesiredMode.TWO_BALL_B);
        mModeChooser.addOption("Two Ball C", DesiredMode.TWO_BALL_C);

        mModeChooser.addOption("Four Ball B", DesiredMode.FOUR_BALL_B);
        mModeChooser.addOption("Four Ball C", DesiredMode.FOUR_BALL_C);

        mModeChooser.addOption("Five Ball", DesiredMode.FIVE_BALL);
        mModeChooser.addOption("One Ball A/B", DesiredMode.ONE_BALL_A_B);
        mModeChooser.addOption("One Ball C/Border", DesiredMode.ONE_BALL_C_BORDER);

        SmartDashboard.putData("Auto mode", mModeChooser);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);
    }

    public void setHardwareFailure(boolean hasFailed) {
        if (RobotBase.isReal()) {
            hardwareFailure = hasFailed;
        }
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (
            mCachedDesiredMode != desiredMode ||
            startingPosition != mCachedStartingPosition
        ) {
            System.out.println(
                "Auto selection changed, updating creator: desiredMode->" +
                desiredMode.name() +
                ", starting position->" +
                startingPosition.name()
            );
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.LEFT_HAB_2
        );
    }

    private boolean startingHab1(StartingPosition position) {
        return (
            position == StartingPosition.LEFT_HAB_1 ||
            position == StartingPosition.RIGHT_HAB_1
        );
    }

    private Optional<AutoModeBase> getAutoModeForParams(
        DesiredMode mode,
        StartingPosition position
    ) {
        if (hardwareFailure) {
            return Optional.of(new DriveStraightMode());
        }
        switch (mode) {
            // 2020
            case DRIVE_STRAIGHT:
                return (Optional.of(new DriveStraightMode()));
            case DRIVE_STRAIGHT_SHOOT:
                return Optional.of(new DriveStraightShootMode());
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case TUNE_DRIVETRAIN:
                return Optional.of(new TuneDrivetrainMode());
            case LIVING_ROOM:
                return (Optional.of(new LivingRoomMode()));
            case TWO_BALL_A:
                return (Optional.of(new TwoBallModeA()));
            case TWO_BALL_B:
                return (Optional.of(new TwoBallModeB()));
            case FOUR_BALL_B:
                return (Optional.of(new FourBallModeB()));
            case TWO_BALL_C:
                return (Optional.of(new TwoBallModeC()));
            case FOUR_BALL_C:
                return (Optional.of(new FourBallModeC()));
            case FIVE_BALL:
                return (Optional.of(new FiveBallMode()));
            case ONE_BALL_A_B:
                return (Optional.of(new OneBallA_BMode()));
            case ONE_BALL_C_BORDER:
                return (Optional.of(new OnceBallC_BorderMode()));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString(
            "StartingPositionSelected",
            mCachedStartingPosition.name()
        );
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}
