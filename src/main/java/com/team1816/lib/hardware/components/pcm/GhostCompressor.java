package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;

public class GhostCompressor implements ICompressor {

    private boolean enabled = false;

    @Override
    public void start() {
        this.enabled = true;
    }

    @Override
    public void stop() {
        this.enabled = false;
    }

    @Override
    public boolean enabled() {
        return enabled;
    }

    @Override
    public double getCompressorCurrent() {
        return 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Compressor");
        builder.addBooleanProperty(
            "Enabled",
            this::enabled,
            value -> {
                if (value) {
                    start();
                } else {
                    stop();
                }
            }
        );
    }

    @Override
    public void close() throws Exception {}
}
