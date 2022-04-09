package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Spindexer extends Subsystem {

    private static final String NAME = "spindexer";

    // Components
    private final ISolenoid feederFlap;
    private final IMotorControllerEnhanced spindexer;

    // State
    private STATE state = STATE.STOP;
    private boolean feederFlapOut = false; // leave for future addition if needed
    private boolean distanceManaged = false;
    private double spindexerPower;
    private boolean outputsChanged;

    // Constants
    private final double COLLECT;
    private final double INDEX;
    private final double FLUSH;
    private final double FIRE;
    private final double POWER_THRESHOLD;

    public Spindexer() {
        super(NAME);
        this.feederFlap = factory.getSolenoid(NAME, "feederFlap");
        this.spindexer = factory.getMotor(NAME, "spindexer");

        COLLECT = factory.getConstant(NAME, "collectPow", 0.5);
        INDEX = factory.getConstant(NAME, "indexPow", -0.25);
        FLUSH = factory.getConstant(NAME, "flushPow", -1);
        FIRE = factory.getConstant(NAME, "firePow", 1);
        POWER_THRESHOLD = .1;
    }

    private void setSpindexer(double spindexerPower) {
        this.spindexerPower = spindexerPower;
        System.out.println(spindexerPower + " = spindexer pow");
        spindexer.set(ControlMode.PercentOutput, spindexerPower);
    }

    private void lockToShooter() { // bear in mind this might never fire if shooter not implemented - not rly important tho
        if (robotState.shooterState == Shooter.STATE.REVVING) {
            setSpindexer(FIRE);
        } else {
            outputsChanged = true; // keep looping through writeToHardWare if shooter not up to speed
        }
    }

    public void setFeederFlap(boolean feederFlapOut) {
        this.feederFlapOut = feederFlapOut;
        outputsChanged = true;
    }

    public void setDesiredState(STATE state) {
        if (this.state != state) {
            this.state = state;
            outputsChanged = true;
            System.out.println("desired spindexer " + state);
        }
    }

    @Override
    public void readFromHardware() {
        if (state != robotState.spinState) {
            //            double actualVel = spindexer.getSelectedSensorVelocity(0);

            //            if(state == SPIN_STATE.COLLECT || state == SPIN_STATE.INDEX){ // logic not yet made for collect / index
            //                robotState.spinState = state;
            //            } else if(Math.abs(spindexerPower) < POWER_THRESHOLD) {
            //                robotState.spinState = SPIN_STATE.STOP;
            //            } else if (spindexerPower > POWER_THRESHOLD) {
            //                robotState.spinState = SPIN_STATE.FIRE;
            //            } else if(spindexerPower < -POWER_THRESHOLD){
            //                robotState.spinState = SPIN_STATE.FLUSH;
            //            }
            robotState.spinState = state;

            System.out.println("ACTUAL SPINDEXER STATE = " + robotState.spinState);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (state) {
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
                    lockToShooter();
                    break;
            }
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
    }
}
