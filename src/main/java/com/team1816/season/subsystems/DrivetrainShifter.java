package com.team1816.season.subsystems;


import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

@Singleton
public class DrivetrainShifter extends Subsystem {

    private static final String NAME = "drivetrainShifter";
    private boolean outputsChanged = false;

    private boolean isShifted = false;
    private STATE desiredState = STATE.STOP;

    // Components
    private final ISolenoid shifter;

    //Actual Stuff

    public DrivetrainShifter() {
        super(NAME);
        shifter = factory.getSolenoid(NAME,"drivetrainShifter");
    }

    public void sprinting(boolean sprint){
        if(sprint){
            this.setDesiredState(STATE.HIGHGEAR);
        }
        else {
            this.setDesiredState(STATE.LOWGEAR);
        }
    }
    public void setDesiredState(STATE state) {
        if (desiredState != state) {
            desiredState = state;
            outputsChanged = true;
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP:
                    isShifted = false;
                    break;
                case LOWGEAR:
                    isShifted = false;
                    break;
                case HIGHGEAR:
                    isShifted = true;
                    break;
            }
            shifter.set(isShifted);
            }
    }

    @Override
    public void stop() {
        this.setDesiredState(STATE.STOP);

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        STOP,
        LOWGEAR,
        HIGHGEAR,
    }
}
