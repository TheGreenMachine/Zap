package com.team1816.lib.hardware.components.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class GhostDigitalInput extends DigitalInput {

    //states
    private boolean on;

    @Override
    public boolean get() {
        return on;
    }

    @Override
    public int getChannel() {
        return -1;
    }

    public GhostDigitalInput(int channel) {
        super(channel);
    }
}
