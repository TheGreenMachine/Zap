package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class GhostDoubleSolenoid implements IDoubleSolenoid {

    // State
    private Value value;

    @Override
    public Value get() {
        return value;
    }

    @Override
    public void set(Value value) {
        this.value = value;
    }

    @Override
    public void toggle() {
        if (value == Value.kForward) {
            set(Value.kReverse);
        } else if (value == Value.kReverse) {
            set(Value.kForward);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Double Solenoid");
        builder.setActuator(true);
        builder.setSafeState(() -> set(Value.kOff));
        builder.addStringProperty(
            "Value",
            () -> get().name().substring(1),
            value -> {
                if ("Forward".equals(value)) {
                    set(Value.kForward);
                } else if ("Reverse".equals(value)) {
                    set(Value.kReverse);
                } else {
                    set(Value.kOff);
                }
            }
        );
    }

    @Override
    public void close() throws Exception {}
}
