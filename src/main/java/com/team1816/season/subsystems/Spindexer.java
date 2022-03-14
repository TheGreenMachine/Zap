package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

@Singleton
public class Spindexer extends Subsystem {

    private static final String NAME = "spindexer";

    // Components
    private final ISolenoid feederFlap;
    private final IMotorControllerEnhanced spindexer;

    // State
    private SPIN_STATE state = SPIN_STATE.STOP;
    private boolean feederFlapOut = false; // leave for future addition if needed
    private boolean distanceManaged = false;
    private double spindexerPower;
    private boolean outputsChanged;

    // Constants
    private final double COLLECT;
    private final double INDEX;
    private final double FLUSH;
    private final double FIRE;

    public Spindexer() {
        super(NAME);
        this.feederFlap = factory.getSolenoid(NAME, "feederFlap");
        this.spindexer = factory.getMotor(NAME, "spindexer");

        COLLECT = factory.getConstant(NAME, "collectPow", 0.5);
        INDEX = factory.getConstant(NAME, "indexPow", -0.25);
        FLUSH = factory.getConstant(NAME, "flushPow", -1);
        FIRE = factory.getConstant(NAME, "firePow", 1);
    }

    public void setSpindexer(double spindexerPower) {
        this.spindexerPower = spindexerPower;
        outputsChanged = true;
    }

    public void setFeederFlap(boolean feederFlapOut) {
        this.feederFlapOut = feederFlapOut;
        outputsChanged = true;
    }

    public void setDesiredState(SPIN_STATE state) {
        if (this.state != state) {
            this.state = state;
            switch (state) {
                case STOP:
                    setSpindexer(0);
                    robotState.spinState = SPIN_STATE.STOP;
                    break;
                case COLLECT:
                    robotState.spinState = SPIN_STATE.COLLECT;
                    setSpindexer(COLLECT);
                    break;
                case INDEX:
                    robotState.spinState = SPIN_STATE.INDEX;
                    setSpindexer(INDEX);
                    break;
                case FLUSH:
                    robotState.spinState = SPIN_STATE.FLUSH;
                    setSpindexer(FLUSH);
                    break;
                case FIRE:
                    break;
            }
            System.out.println("DESIRED SPINDEXER STATE = " + state);
        }
    }

    @Override
    public void readFromHardware() {
        if (
            robotState.shooterState == Shooter.SHOOTER_STATE.REVVING &&
            state != robotState.spinState
        ) {
            if (spindexerPower != FIRE) {
                System.out.println("ACTUAL SPINDEXER STATE = FIRE");
            }
            robotState.spinState = SPIN_STATE.FIRE;
            setSpindexer(FIRE);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            System.out.println(spindexerPower + " = spindexer pow");
            spindexer.set(ControlMode.PercentOutput, spindexerPower);
            this.feederFlap.set(feederFlapOut);
            distanceManaged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {}

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    public enum SPIN_STATE {
        STOP,
        COLLECT,
        INDEX,
        FLUSH,
        FIRE,
    }
}
