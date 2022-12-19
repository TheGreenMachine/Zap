package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem to indexes game elements
 */
@Singleton
public class Spindexer extends Subsystem {

    /**
     * Properties
     */
    private static final String NAME = "spindexer";

    /**
     * Components
     */
    private final IGreenMotor spindexer;

    /**
     * State
     */
    private STATE desiredState = STATE.STOP;
    private double desiredPower;
    private boolean outputsChanged;

    private final double COLLECT;
    private final double INDEX;
    private final double FLUSH;
    private final double FIRE;
    private final double COAST;

    /**
     * Instantiates a spindexer with base subsystem properties
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Spindexer(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        // Components
        spindexer = factory.getMotor(NAME, "spindexer");

        // Constants
        COLLECT = factory.getConstant(NAME, "collectPow", 0.5);
        INDEX = factory.getConstant(NAME, "indexPow", -0.25);
        FLUSH = factory.getConstant(NAME, "flushPow", -1);
        FIRE = factory.getConstant(NAME, "firePow", 1);
        COAST = factory.getConstant(NAME, "coastPow", -.1);
    }

    /** Actions */

    /**
     * Sets the spindexer based on demand
     * @param spindexerPower demand
     */
    private void setSpindexer(double spindexerPower) {
        this.desiredPower = spindexerPower;
        spindexer.set(ControlMode.PercentOutput, spindexerPower);
    }

    /**
     * Feeds ball into elevator if firing otherwise indexes
     */
    private void lockToElevator() {
        if (robotState.elevatorState == Elevator.STATE.FIRE) {
            setSpindexer(FIRE);
        } else {
            setSpindexer(INDEX);
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired state of the spindexer
     * @param state STATE
     */
    public void setDesiredState(STATE state) {
        if (desiredState != state) {
            desiredState = state;
            outputsChanged = true;
        }
    }

    /** Periodic */

    /**
     * Updates RobotState
     */
    @Override
    public void readFromHardware() {
        if (desiredState != robotState.spinState) {
            robotState.spinState = desiredState;
        }
    }

    /**
     * Sends outputs to the spindexerMotor based on desiredState
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP:
                    setSpindexer(0);
                    break;
                case COLLECT:
                    setSpindexer(COLLECT);
                    break;
                case INDEX:
                    setSpindexer(INDEX);
                    break;
                case FLUSH:
                    setSpindexer(FLUSH);
                    break;
                case FIRE:
                    lockToElevator();
                    break;
                case COAST:
                    setSpindexer(COAST);
                    break;
            }
        }
    }

    /** Config and Tests */

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {}

    /**
     * Initializes a SendableBuilder for SmartDhasboard
     * @param builder SendableBuilder
     */
    @Override
    public void initSendable(SendableBuilder builder) {}

    /**
     * Stops the spindexer
     */
    @Override
    public void stop() {}

    /**
     * Tests the subsystem
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        spindexer.set(ControlMode.PercentOutput, .3);
        Timer.delay(1);
        spindexer.set(ControlMode.PercentOutput, -.3);
        Timer.delay(1);
        spindexer.set(ControlMode.PercentOutput, 0);
        return true;
    }

    /**
     * Base enum for spindexer states
     */
    public enum STATE {
        STOP,
        COLLECT,
        INDEX,
        FLUSH,
        FIRE,
        COAST,
    }
}
