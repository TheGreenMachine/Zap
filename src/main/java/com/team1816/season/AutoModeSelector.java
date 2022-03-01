package com.team1816.season;

import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import javax.inject.Singleton;
import javax.swing.text.html.Option;

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
        DRIVE_BY_CAMERA,
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        TURRET_TEST,
        LIVING_ROOM,
        DRIVE_STRAIGHT,

        // 2022
        TWO_BALL_A,
        TWO_BALL_B,
        TWO_BALL_C,
        THREE_BALL_A,
        THREE_BALL_B,
        THREE_BALL_C,
        FOUR_BALL_SEMI_A,
        FOUR_BALL_SEMI_B,
        FOUR_BALL_C,
        FIVE_BALL,
        RANDOM_TESTING_PATH
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

        // 2020

        mModeChooser.addOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        mModeChooser.addOption("Tune Drivetrain", DesiredMode.TUNE_DRIVETRAIN);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        SmartDashboard.putData("Auto mode", mModeChooser);

        // CheezeCurd
        //        mModeChooser.addOption("Shop", DesiredMode.SHOP);
        //        mModeChooser.addOption("PID", DesiredMode.PID);
        mModeChooser.setDefaultOption("Drive Straight", DesiredMode.DRIVE_STRAIGHT);
        mModeChooser.addOption("Turret Tuning", DesiredMode.TURRET_TEST);
        mModeChooser.addOption("Living Room", DesiredMode.LIVING_ROOM);

        mModeChooser.addOption("Two Ball A", DesiredMode.TWO_BALL_A);
        mModeChooser.addOption("Two Ball B", DesiredMode.TWO_BALL_B);
        mModeChooser.addOption("Two Ball C", DesiredMode.TWO_BALL_C);

        mModeChooser.addOption("Three Ball A", DesiredMode.THREE_BALL_A);
        mModeChooser.addOption("Three Ball B", DesiredMode.THREE_BALL_B);
        mModeChooser.addOption("Three Ball C", DesiredMode.THREE_BALL_C);

        mModeChooser.addOption("Four Ball Semicircle A", DesiredMode.FOUR_BALL_SEMI_A);
        mModeChooser.addOption("Four Ball Semicircle B", DesiredMode.FOUR_BALL_SEMI_B);
        mModeChooser.addOption("Four Ball C", DesiredMode.FOUR_BALL_C);

        mModeChooser.addOption("Five Ball", DesiredMode.FIVE_BALL);
        mModeChooser.addOption("Random Testing Path", DesiredMode.RANDOM_TESTING_PATH);

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
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case TUNE_DRIVETRAIN:
                return Optional.of(new TuneDrivetrainMode());
            case TURRET_TEST:
                return Optional.of(new TurretTestMode());
            case DRIVE_STRAIGHT:
                return (Optional.of(new DriveStraightMode()));
            case LIVING_ROOM:
                return (Optional.of(new LivingRoomMode()));
            case TWO_BALL_A:
                return (Optional.of(new TwoBallModeA()));
            case TWO_BALL_B:
                return (Optional.of(new TwoBallModeB()));
            case TWO_BALL_C:
                return (Optional.of(new TwoBallModeC()));
            case THREE_BALL_A:
                return (Optional.of(new ThreeBallModeA()));
            case THREE_BALL_B:
                return (Optional.of(new ThreeBallModeB()));
            case THREE_BALL_C:
                return (Optional.of(new ThreeBallModeC()));
            case FOUR_BALL_SEMI_A:
                return (Optional.of(new FourBallSemiCircleModeA()));
            case FOUR_BALL_SEMI_B:
                return (Optional.of(new FourBallSemiCircleModeB()));
            case FOUR_BALL_C:
                return (Optional.of(new FourBallModeC()));
            case FIVE_BALL:
                Constants.StartingPose = new FiveBallMode().startingPose;
                return (Optional.of(new FiveBallMode()));
            case RANDOM_TESTING_PATH:
                Constants.StartingPose = new FiveBallMode().startingPose;
                return (Optional.of(new RandomTestingMode()));
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

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }

    private static AutoModeSelector INSTANCE;
}
