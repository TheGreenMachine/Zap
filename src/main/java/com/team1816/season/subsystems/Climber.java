package com.team1816.season.subsystems;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.ControlMode.Position;

import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem {

    // this class uses some logic from the turret (positionControl logic) and from the distanceManager (Stage behaves like bucket Entries)
    private static final String NAME = "climber";

    // Components
    private final IGreenMotor climberMain;
    private final IGreenMotor climberFollower;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    // State
    private ControlMode controlMode = ControlMode.MANUAL;
    private double error;
    private boolean unlocked;
    private boolean needsOverShoot = false;
    private boolean needsClamp = false;
    private boolean climbDelay = false;
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

    public Climber() {
        super(NAME);
        climberMain = factory.getMotor(NAME, "climberMain");
        climberFollower =
            (IGreenMotor) factory.getMotor(NAME, "climberFollower", climberMain);
        climberFollower.setInverted(true);
        topClamp = factory.getSolenoid(NAME, "topClamp");
        bottomClamp = factory.getSolenoid(NAME, "bottomClamp");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        ALLOWABLE_ERROR = config.allowableError;

        climberMain.configClosedloopRamp(.20, Constants.kCANTimeoutMs);
        climberFollower.configClosedloopRamp(.20, Constants.kCANTimeoutMs);

        currentStage = 0;
        unlocked = false;

        stages =
            new Stage[] {
                new Stage(factory.getConstant(NAME, "startPos", 0), true, false, false),
                new Stage(
                    factory.getConstant(NAME, "unlockPos", -20),
                    true,
                    false,
                    false
                ),
                new Stage(factory.getConstant(NAME, "returnPos", 0), true, false, false),
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

    public void unlock() {
        unlocked = true;
    }

    public void incrementClimberStage() { // we can't go backwards (descend rungs) using this logic, but it shouldn't really matter
        if (!unlocked) {
            System.out.println("climber not unlocked!");
            return;
        }
        if (currentStage < stages.length - 1 && (!needsOverShoot || currentStage == 2)) {
            if (controlMode != ControlMode.POSITION) {
                controlMode = ControlMode.POSITION;
            }
            System.out.println(
                "incrementing climber to stage " + currentStage + " ....."
            );
            currentStage++;
            needsOverShoot = true;
            climbDelay = true;
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
            climberMain.set(Position, position);
            if (climbDelay) {
                climbDelay = false;
                Timer.delay(1);
            }
            if (Math.abs(error) < ALLOWABLE_ERROR && currentStage != 2) {
                needsOverShoot = false;
            }
            outputsChanged = true;
        } else {
            climberMain.set(PercentOutput, 0); // coast so that the climber falls down to lock
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
        error = climberMain.getSelectedSensorPosition(0) - stages[currentStage].position;
        climberPosition = climberMain.getSelectedSensorPosition(0);
        //        currentDraw = climberMain.getOutputCurrent();
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
                climberMain.set(PercentOutput, climberPower);
            }
        }
    }

    public double getClimberPosition() {
        return climberPosition;
    }

    public double getCurrentDraw() {
        return currentDraw;
    }

    public int getCurrentStage() {
        return currentStage;
    }

    @Override
    public void zeroSensors() {
        currentStage = 0;
        unlocked = false;
        needsClamp = false;
        climbDelay = false;
        needsOverShoot = false;
        climberMain.setSelectedSensorPosition(
            stages[0].position,
            0,
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        // currently just running the climber in percent output to make sure it can unclamp and spin
        // WARNING - pos/neg values may be inverted!
        climberMain.set(PercentOutput, 0.2);
        Timer.delay(.5);
        climberMain.set(PercentOutput, 0);
        Timer.delay(1);
        climberMain.set(PercentOutput, -0.2);
        Timer.delay(2);
        climberMain.set(PercentOutput, 0);

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
