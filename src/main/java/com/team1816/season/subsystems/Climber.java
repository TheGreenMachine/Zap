package com.team1816.season.subsystems;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.ControlMode.Position;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem that models a "windmill" style climber
 */
@Singleton
public class Climber extends Subsystem {

    /**
     * Properties
     */
    private static final String NAME = "climber";

    /**
     * Components
     */
    private final IGreenMotor climberMain;
    private final IGreenMotor climberFollower;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    /**
     * State
     */
    private ControlMode controlMode = ControlMode.MANUAL;
    private double error;
    private boolean unlocked;
    private boolean needsOverShoot = false;
    private boolean needsClamp = false;
    private boolean climbDelay = false;

    private int currentStage;
    private final Stage[] stages;
    private double climberPosition;

    private double climberPower = 0;
    private boolean topClamped = false;
    private boolean bottomClamped = false;
    private boolean outputsChanged = false;

    private final double ALLOWABLE_ERROR;

    /**
     * Instantiates a climber from base subsystem properties
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Climber(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        climberMain = factory.getMotor(NAME, "climberMain");
        climberFollower = factory.getFollowerMotor(NAME, "climberFollower", climberMain);
        climberFollower.setInverted(!climberMain.getInverted());
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

    /** Actions */

    /**
     * Unlocks the climber
     */
    public void unlock() {
        System.out.println("Unlocking Climber!");
        unlocked = true;
    }

    /**
     * Increments climber stage
     */
    public void incrementClimberStage() {
        if (!unlocked) {
            System.out.println("Climber NOT Unlocked!");
            return;
        }
        if (currentStage < stages.length - 1 && (!needsOverShoot || currentStage == 2)) {
            if (controlMode != ControlMode.POSITION) {
                controlMode = ControlMode.POSITION;
            }
            System.out.println(
                "Incrementing Climber To Stage " + (currentStage + 1) + " ....."
            );
            currentStage++;
            needsOverShoot = true;
            climbDelay = true;
            needsClamp = true;
            outputsChanged = true;
        } else {
            System.out.println(
                "Climber NOT safely at stage " +
                currentStage +
                " - not incrementing stage!"
            );
        }
    }

    /**
     * Sets the climber power based on demand
     * @param power demand
     */
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

    /**
     * Toggles the top clamp
     */
    public void setTopClamp() {
        if (!unlocked) {
            System.out.println("Climber NOT Unlocked!");
            return;
        }
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        topClamped = !topClamped;
        needsClamp = true;
        outputsChanged = true;
    }

    /**
     * Toggles the bottom clamp
     */
    public void setBottomClamp() {
        if (!unlocked) {
            System.out.println("Climber NOT Unlocked!");
            return;
        }
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        bottomClamped = !bottomClamped;

        needsClamp = true;
        outputsChanged = true;
    }

    /**
     * Sets the climber to a desired position
     * @param position desiredPosition
     */
    private void positionControl(double position) {
        if (needsOverShoot) {
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
            climberMain.set(PercentOutput, 0);
        }
    }

    /**
     * Sets the top and bottom clamps based on desired states and order
     * @param topClamped boolean
     * @param bottomClamped boolean
     * @param topFirst boolean clampTopFirst
     */
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

    /**
     * Returns the climber position
     * @return position
     */
    public double getClimberPosition() {
        return climberPosition;
    }

    /**
     * Returns the current stage
     * @return stage index
     */
    public int getCurrentStage() {
        return currentStage;
    }

    /** Periodic */

    /**
     * Reads from motor values and updates state
     */
    @Override
    public void readFromHardware() {
        error = climberMain.getSelectedSensorPosition(0) - stages[currentStage].position;
        climberPosition = climberMain.getSelectedSensorPosition(0);
    }

    /**
     * Writes outputs to the motor based on controlMode and desired state
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            if (controlMode == ControlMode.POSITION) {
                Stage stage = stages[currentStage];
                setClamps(stage.topClamped, stage.bottomClamped, stage.topFirst);
                positionControl(stages[currentStage].position); // gets current position in profile
            } else {
                setClamps(topClamped, bottomClamped, false);
                climberMain.set(PercentOutput, climberPower);
            }
        }
    }

    /** Config and Tests */

    /**
     * Zeroes the climber motor position
     */
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

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {}

    /**
     * Tests the subsystem
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        climberMain.set(PercentOutput, 0.2);
        Timer.delay(.5);
        climberMain.set(PercentOutput, 0);
        Timer.delay(1);
        climberMain.set(PercentOutput, -0.2);
        Timer.delay(2);
        climberMain.set(PercentOutput, 0);

        return true;
    }

    /** Modes and Stages */

    /**
     * Base enum for control modes
     */
    public enum ControlMode {
        MANUAL,
        POSITION,
    }

    /**
     * Base static class for climber stages
     */
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
