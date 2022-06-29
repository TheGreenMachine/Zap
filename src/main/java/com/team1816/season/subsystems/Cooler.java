package com.team1816.season.subsystems;

import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import javax.inject.Singleton;

/*
this is an example of a subsystem that's currently simple enough that it needs no marked difference between desired and actual states
- no other subsystems depend on this subsystem in order to perform an action
 */
@Singleton
public class Cooler extends Subsystem {

    private static final String NAME = "cooler";

    // Components
    private final ISolenoid dumpIn;
    private final ISolenoid dumpOut;

    private boolean needsDump = false;
    private boolean outputsChanged = false;

    private AsyncTimer coolTimer;

    public Cooler() {
        super(NAME);
        dumpIn = factory.getSolenoid(NAME, "dumpIn");
        dumpOut = factory.getSolenoid(NAME, "dumpOut");

        coolTimer = new AsyncTimer(0.5, () -> dumpIn.set(needsDump), () -> dumpOut.set(!needsDump));
    }

    @Override
    public void readFromHardware() {
        if (robotState.overheating != needsDump) {
            needsDump = robotState.overheating;
            outputsChanged = true;
        }
        if (dumpIn.get()) {
            robotState.coolState = STATE.DUMP;
        } else {
            robotState.coolState = STATE.WAIT;
        }
    }

    @Override
    public void writeToHardware() {
        if(outputsChanged){
            outputsChanged = false;
            coolControl();
        }
    }

    public void coolControl(){
        if(!coolTimer.isCompleted()){
            coolTimer.update();
            outputsChanged = true;
        } else {
            coolTimer.reset();
        }
    }

    @Override
    public void zeroSensors() {
        needsDump = false;
        outputsChanged = false;
        coolTimer.reset();
    }

    @Override
    public void stop() {
        dumpIn.set(false);
        dumpOut.set(false);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        WAIT,
        DUMP,
    }
}
