package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Spindexer extends Subsystem {

    private static final String NAME = "spindexer";

    // Components
    private final IGreenMotor spindexer;

    // State
    private STATE desiredState = STATE.STOP;
    private double desiredPower;
    private boolean outputsChanged;

    // Constants
    private final double COLLECT;
    private final double INDEX;
    private final double FLUSH;
    private final double FIRE;
    private final double COAST;

    public Spindexer() {
        super(NAME);
        // Components
        spindexer = factory.getMotor(NAME, "spindexer");

        // Constants
        COLLECT = factory.getConstant(NAME, "collectPow", 0.5);
        INDEX = factory.getConstant(NAME, "indexPow", -0.25);
        FLUSH = factory.getConstant(NAME, "flushPow", -1);
        FIRE = factory.getConstant(NAME, "firePow", 1);
        COAST = factory.getConstant(NAME, "coastPow", -.1);
    }

    private void setSpindexer(double spindexerPower) {
        this.desiredPower = spindexerPower;
        spindexer.set(ControlMode.PercentOutput, spindexerPower);
    }

    private void lockToElevator() { // bear in mind this might never fire if shooter not implemented - not rly important tho
        if (robotState.elevatorState == Elevator.STATE.FIRE) {
            setSpindexer(FIRE);
        } else {
            setSpindexer(INDEX);
            outputsChanged = true; // keep looping through writeToHardWare if shooter not up to speed
        }
    }

    public void setDesiredState(STATE state) {
        if (desiredState != state) {
            desiredState = state;
            outputsChanged = true;
        }
    }

    @Override
    public void readFromHardware() {
        // since no other subsystems rely on spindexer being up to speed to perform an action,
        // we're just claiming that the true spindexer state matches its desired state
        if (desiredState != robotState.spinState) {
            robotState.spinState = desiredState;
        }
    }

    @Override
    public void writeToHardware() {
        // avoid setting the spindexer motor unless outputs (ie: state) are changed
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

    @Override
    public void initSendable(SendableBuilder builder) {}

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        spindexer.set(ControlMode.PercentOutput, .3);
        Timer.delay(1);
        spindexer.set(ControlMode.PercentOutput, -.3);
        Timer.delay(1);
        spindexer.set(ControlMode.PercentOutput, 0);
        return true;
    }

    public enum STATE {
        STOP,
        COLLECT,
        INDEX,
        FLUSH,
        FIRE,
        COAST,
    }
}
