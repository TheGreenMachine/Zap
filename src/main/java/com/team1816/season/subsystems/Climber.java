package com.team1816.season.subsystems;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.ControlMode.Position;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem {

    // this class uses some logic from the turret (positionControl logic) and from the distanceManager (Stage behaves like bucket Entries)
    private static final String NAME = "climber";

    // Components
    private final IMotorControllerEnhanced spinner;
    private final IMotorControllerEnhanced spinnerFollower;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    // State
    private ControlMode controlMode = ControlMode.MANUAL;
    private double error;
    private boolean unlocked;
    private boolean needsOverShoot = false;
    private boolean needsClamp = false;
    // Position
    private int currentStage;
    private final Stage[] stages;
    private double climberPosition;
    private double currentDraw;
    //Manual
    private double climberPower = 0;
    private boolean topClamped = false;
    private boolean bottomClamped = false;
    private boolean outputsChanged = false;

    private final double ALLOWABLE_ERROR;
    private final String pidSlot = "slot0";

    public Climber() {
        super(NAME);
        spinner = factory.getMotor(NAME, "spinner");
        spinnerFollower =
            (IMotorControllerEnhanced) factory.getMotor(
                NAME,
                "spinnerFollower",
                spinner,
                true
            );
        topClamp = factory.getSolenoid(NAME, "topClamp");
        bottomClamp = factory.getSolenoid(NAME, "bottomClamp");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME, pidSlot);

        ALLOWABLE_ERROR = config.allowableError;

        spinner.config_kP(0, config.kP, 100);
        spinner.config_kI(0, config.kI, 100);
        spinner.config_kD(0, config.kD, 100);
        spinner.config_kF(0, config.kF, 100);

        spinnerFollower.config_kP(0, config.kP, 100);
        spinnerFollower.config_kI(0, config.kI, 100);
        spinnerFollower.config_kD(0, config.kD, 100);
        spinnerFollower.config_kF(0, config.kF, 100);

        spinner.configClosedloopRamp(.20, Constants.kCANTimeoutMs);
        spinnerFollower.configClosedloopRamp(.20, Constants.kCANTimeoutMs);

        currentStage = 0;
        unlocked = false;

        stages =
            new Stage[] {
                new Stage(factory.getConstant(NAME, "startPos", 0), true, false, false), // just here as an init value
                new Stage(
                    factory.getConstant(NAME, "unlockPos", -20),
                    true,
                    false,
                    false
                ),
                new Stage(factory.getConstant(NAME, "startPos", 0), true, false, false),
                new Stage(
                    factory.getConstant(NAME, "firstToSecondRungPos", -63),
                    false,
                    true,
                    true
                ),
                new Stage(
                    factory.getConstant(NAME, "secondToLastRungPos", -153),
                    true,
                    false,
                    false
                ),
                new Stage(factory.getConstant(NAME, "lastPos", -180), false, true, true),
            };
    }

    public void setUnlocked() {
        unlocked = !unlocked;
    }

    public void incrementClimberStage() { // we can't go backwards (descend rungs) using this logic, but it shouldn't really matter
        if (!unlocked) {
            System.out.println("climber not unlocked!");
            return;
        }
        if (currentStage < stages.length - 1 && !needsOverShoot) {
            if (controlMode != ControlMode.POSITION) {
                controlMode = ControlMode.POSITION;
            }
            System.out.println(
                "incrementing climber to stage " + currentStage + " ....."
            );
            currentStage++;
            needsOverShoot = true;
            needsClamp = true;
            outputsChanged = true;
        } else {
            System.out.println(
                "climber not safely at stage " +
                currentStage +
                " - not incrementing stage!"
            );
        }
    }

    public void setClimberPower(double power) {
        if (!unlocked) {
            System.out.println("climber not unlocked!");
            return;
        }
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        climberPower = power;
        outputsChanged = true;
    }

    public void setTopClamp() {
        if (!unlocked) {
            System.out.println("climber not unlocked!");
            return;
        }
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        topClamped = !topClamped;
        needsClamp = true;
        outputsChanged = true;
    }

    public void setBottomClamp() {
        if (!unlocked) {
            System.out.println("climber not unlocked!");
            return;
        }
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        bottomClamped = !bottomClamped;

        needsClamp = true;
        outputsChanged = true;
    }

    private void positionControl(double position) {
        if (needsOverShoot) { // keep looping if we aren't past the overshoot value
            spinner.set(Position, position);
            if (Math.abs(error) < 5) {
                needsOverShoot = false;
            }
            outputsChanged = true;
        } else {
            spinner.set(PercentOutput, 0); // coast so that the climber falls down to lock
        }
    }

    private void setClamps(boolean topClamped, boolean bottomClamped, boolean topFirst) {
        if (needsClamp) {
            needsClamp = false;
            if (topFirst) {
                topClamp.set(topClamped);
                Timer.delay(.25);
                bottomClamp.set(bottomClamped);
            } else {
                bottomClamp.set(bottomClamped);
                Timer.delay(.25);
                topClamp.set(topClamped);
            }
            System.out.println("setting climber clamps!");
            Timer.delay(.25);
        }
    }

    @Override
    public void readFromHardware() {
        error = spinner.getSelectedSensorPosition(0) - stages[currentStage].position;
        climberPosition = spinner.getSelectedSensorPosition(0);
        currentDraw = spinner.getOutputCurrent();
        //        System.out.println("climber position = " + climberPosition);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            if (controlMode == ControlMode.POSITION) {
                Stage stage = stages[currentStage];
                setClamps(stage.topClamped, stage.bottomClamped, stage.topFirst);
                positionControl(stages[currentStage].position);
            } else {
                setClamps(topClamped, bottomClamped, false);
                spinner.set(PercentOutput, climberPower);
            }
        }
    }

    public double getClimberPosition() {
        return climberPosition;
    }

    public double getCurrentDraw() {
        return currentDraw;
    }

    public int getCurrentStage() { return currentStage; }

    @Override
    public void zeroSensors() {
        currentStage = 0;
        spinner.setSelectedSensorPosition(stages[0].position, 0, Constants.kCANTimeoutMs);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    public enum ControlMode {
        MANUAL,
        POSITION,
    }

    static class Stage {

        public final double position;
        public final boolean topClamped;
        public final boolean bottomClamped;
        public final boolean topFirst;

        Stage(
            double distance,
            boolean topClamped,
            boolean bottomClamped,
            boolean topFirst
        ) {
            this.position = distance;
            this.topClamped = topClamped;
            this.bottomClamped = bottomClamped;
            this.topFirst = topFirst;
        }

        Stage() {
            this(0, false, false, false);
        }
    }
}
