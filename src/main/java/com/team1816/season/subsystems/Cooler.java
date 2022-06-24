package com.team1816.season.subsystems;

import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import javax.inject.Singleton;

/*
this is an example of a subsystem that's currently simple enough that it needs no marked difference between desired and actual states
- no other subsystems depend on this subsystem in order to perform an action
 */
@Singleton
public class Cooler extends Subsystem {

    private static final String NAME = "Cooler";

    // Components
    private final ISolenoid dumpIn;
    private final ISolenoid dumpOut;

    private boolean needsDump = false;

    public Cooler() {
        super(NAME);
        dumpIn = factory.getSolenoid(NAME, "dumpIn");
        dumpOut = factory.getSolenoid(NAME, "dumpOut");
    }

    @Override
    public void readFromHardware() {
        if (robotState.hasOverheated) {
            needsDump = true;
        }
        if (dumpIn.get()) {
            robotState.coolState = STATE.DUMP;
        }
    }

    @Override
    public void writeToHardware() {
        if (needsDump && robotState.coolState == STATE.WAIT) {
            dumpIn.set(true);
            dumpOut.set(true);
        }
    }

    @Override
    public void zeroSensors() {
        needsDump = false;
        dumpIn.set(false);
        dumpOut.set(false);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        WAIT,
        DUMP,
    }
}
