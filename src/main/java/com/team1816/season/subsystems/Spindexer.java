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

    private SPIN_STATE state = SPIN_STATE.STOP;

    // State
    private boolean feederFlapOut = false; // leave for future addition if needed
    private boolean distanceManaged = false;
    private double spindexerPower;
    private boolean outputsChanged;

    public Spindexer() {
        super(NAME);
        this.feederFlap = factory.getSolenoid(NAME, "feederFlap");
        this.spindexer = factory.getMotor(NAME, "spindexer");
    }

    public void setSpindexer(double spindexerPower) {
        distanceManaged = true;
        this.spindexerPower = spindexerPower;
    }

    public void setFeederFlap(boolean feederFlapOut) {
        this.feederFlapOut = feederFlapOut;
        outputsChanged = true;
    }

    public void setState(SPIN_STATE state) {
        this.state = state;
        System.out.println("SOINDEXER STATE IS CHANGED TO " + state);
        outputsChanged = true;
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            switch (state) {
                case STOP:
                    spindexerPower = 0;
                    break;
                case INTAKE:
                    spindexerPower = 0.375;
                    break;
                case INDEX:
                    spindexerPower = -0.25;
                    break;
                case FLUSH:
                    spindexerPower = -1;
                    break;
                case FIRE:
                    if (!distanceManaged) spindexerPower = 1;
                    break;
            }
            spindexer.set(ControlMode.PercentOutput, spindexerPower);
            this.feederFlap.set(feederFlapOut);
            distanceManaged = false;
            outputsChanged = false;
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
        INTAKE,
        STOP,
        INDEX,
        FLUSH,
        FIRE,
    }
}
