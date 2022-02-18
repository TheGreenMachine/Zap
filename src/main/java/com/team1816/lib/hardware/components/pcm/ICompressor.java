package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.Sendable;

public interface ICompressor extends Sendable, AutoCloseable {
    void enableDigital();

    void stop();

    boolean enabled();

    double getCompressorCurrent();

    boolean getCompressorSwitchValue();
}
