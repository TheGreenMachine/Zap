package com.team1816.lib.subsystems;

import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.loops.Looper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to zero, enableDigital, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager() {}

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            System.out.println("SUBSYSTEM: " + s.getSubsystemName());
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void outputToSmartDashboard() {}

    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public void zeroSensors() {
        mAllSubsystems.forEach(Subsystem::zeroSensors);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
        for (Subsystem subsystem : mAllSubsystems) {
            if (!subsystem.isImplemented()) {
                System.out.println(
                    "  Warning: " + subsystem.getSubsystemName() + " is not implemented"
                );
            }
        }
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            // loop through calls assigned by registerEnabledLoops (i.e. in Drive)
            mLoops.forEach(l -> l.onLoop(timestamp));

            // loop through read and write from hardware
            mAllSubsystems.forEach(Subsystem::readFromHardware);
            mAllSubsystems.forEach(Subsystem::writeToHardware);
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            // only loop through read from hardware
            mAllSubsystems.forEach(Subsystem::readFromHardware);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}
