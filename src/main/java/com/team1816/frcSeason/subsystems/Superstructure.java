package com.team1816.frcSeason.subsystems;

import com.team1816.frcSeason.Robot;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.vision.AimingParameters;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;

/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * turret, elevator, arm, and wrist. The superstructure subsystem also uses info from the vision system.
 * <p>
 * Instead of interacting individually with subsystems like the elevator and arm, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 * The Superstructure class also adjusts the overall goal based on the turret and elevator control modes.
 */
public class Superstructure extends Subsystem {

    private static Superstructure mInstance;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();

    public Superstructure() {
        super("superstructure");
    }

    @Override
    public boolean isImplemented() {
        return true;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                    synchronized (Superstructure.this) {}
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Superstructure.this) {}
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isAtDesiredState() {
        return true;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
