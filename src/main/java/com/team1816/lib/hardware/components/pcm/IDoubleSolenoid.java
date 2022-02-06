package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IDoubleSolenoid extends Sendable, AutoCloseable {
    DoubleSolenoid.Value get();
    void set(final DoubleSolenoid.Value value);
    void toggle();
}
