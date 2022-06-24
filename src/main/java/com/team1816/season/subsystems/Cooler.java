package com.team1816.season.subsystems;

import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import javax.inject.Singleton;

@Singleton
public class Cooler extends Subsystem {

    private static final String NAME = "Cooler";

    // Components
    private final ISolenoid dumpIn;
    private final ISolenoid dumpOut;

    public Cooler() {
        super(NAME);
        dumpIn = factory.getSolenoid(NAME, "dumpIn");
        dumpOut = factory.getSolenoid(NAME, "dumpOut");
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        STOP,
        DUMP,
    }
}
