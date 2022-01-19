package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;

public class GhostSolenoid implements ISolenoid {

    // State
    private boolean on;

    @Override
    public boolean get() {
        return on;
    }

    @Override
    public void set(boolean on) {
        this.on = on;
    }

    @Override
    public void toggle() {
        set(!get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Solenoid");
        builder.setActuator(true);
        builder.setSafeState(() -> set(false));
        builder.addBooleanProperty("Value", this::get, this::set);
    }

    @Override
    public void close() throws Exception {}
}
