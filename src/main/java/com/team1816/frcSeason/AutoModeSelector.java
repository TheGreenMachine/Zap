package com.team1816.frcSeason;

import com.team1816.frcSeason.auto.modes.modes2020.*;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.auto.modes.DoNothingMode;
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
        DRIVE_BY_CAMERA,
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        TUNE_DRIVETRAIN_REVERSE,
        TURRET_TEST,
        LIVING_ROOM,
        DRIVE_STRAIGHT,

        // 2021
        BARREL,
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
        mModeChooser.addOption(
            "Tune Drivetrain Reverse",
            DesiredMode.TUNE_DRIVETRAIN_REVERSE
        );
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        SmartDashboard.putData("Auto mode", mModeChooser);

        // CheezeCurd
        mModeChooser.addOption("Living Room", DesiredMode.LIVING_ROOM);
        //        mModeChooser.addOption("Shop", DesiredMode.SHOP);
        //        mModeChooser.addOption("PID", DesiredMode.PID);
        mModeChooser.setDefaultOption("Drive Straight", DesiredMode.DRIVE_STRAIGHT);
        mModeChooser.addOption("Turret Tuning", DesiredMode.TURRET_TEST);

        mModeChooser.addOption("Barrel", DesiredMode.BARREL);

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
                return Optional.of(new TuneDrivetrainMode(false));
            case TUNE_DRIVETRAIN_REVERSE:
                return Optional.of(new TuneDrivetrainMode(true));
            case TURRET_TEST:
                return Optional.of(new TurretTestMode());
            case DRIVE_STRAIGHT:
                return (Optional.of(new DriveStraightMode()));
            case LIVING_ROOM:
                return (Optional.of(new LivingRoomMode()));
            case BARREL:
                return (Optional.of(new BarrelMode()));
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
