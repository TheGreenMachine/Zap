package com.team1816.season.subsystems;

import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

/*
this is an example of a subsystem that's currently simple enough that it needs no marked difference between desired and actual states
- no other subsystems depend on this subsystem in order to perform an action
 */
@Singleton
public class Cooler extends Subsystem {

    private static final String NAME = "cooler";

    // Components
    private final ISolenoid lock;
    private final ISolenoid dump;

    private boolean needsDump = false;
    private boolean outputsChanged = false;
    private boolean shutDown = false;

    private final boolean letAirFlow = true;
    private final boolean blockAirFlow = false;
    private AsyncTimer coolTimer;

    public Cooler() {
        super(NAME);
        lock = factory.getSolenoid(NAME, "lock");
        dump = factory.getSolenoid(NAME, "dump");

        coolTimer =
            new AsyncTimer(0.5, () -> lock.set(!needsDump), () -> dump.set(needsDump));
        SmartDashboard.putBoolean("Drive/Overheating", robotState.overheating);
    }

    @Override
    public void readFromHardware() {
        if (robotState.overheating != needsDump) {
            needsDump = robotState.overheating;
            outputsChanged = true;
        }
        if (dump.get() == blockAirFlow) {
            robotState.coolState = STATE.WAIT;
        } else {
            robotState.coolState = STATE.DUMP;
        }
        if (DriverStation.getMatchTime() > 90) {
            shutDown = true;
            outputsChanged = true;
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            coolControl();
        }
    }

    public void coolControl() {
        if (shutDown) {
            needsDump = false;
            if (!coolTimer.isCompleted()) {
                System.out.println("shutting down cooler");
                coolTimer.update();
                outputsChanged = true;
            }
        } else {
            if (!coolTimer.isCompleted()) {
                coolTimer.update();
                outputsChanged = true;
            } else {
                coolTimer.reset();
                SmartDashboard.putBoolean("Drive/Overheating", robotState.overheating);
            }
        }
    }

    @Override
    public void zeroSensors() {
        needsDump = false;
        outputsChanged = false;
        shutDown = false;
        coolTimer.reset();
    }

    @Override
    public void stop() {
        lock.set(letAirFlow);
        dump.set(blockAirFlow);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum STATE {
        WAIT,
        DUMP,
    }
}
