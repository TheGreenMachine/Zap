package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.util.sendable.Sendable;

public interface IDoubleSolenoid extends Sendable, AutoCloseable {
    DoubleSolenoid.Value get();
    void set(final DoubleSolenoid.Value value);
    void toggle();
}
