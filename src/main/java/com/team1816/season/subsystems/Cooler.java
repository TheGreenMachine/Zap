package com.team1816.season.subsystems;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * This subsystem models a compressed air cooling system for motors to ensure optimal performance and prevent overheating.
 * This is also an example of a subsystem that needs no marked difference between desired and actual states
 * i.e. no other subsystems depend on this subsystem in order to perform an action
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

    @Inject
    public Cooler(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        lock = factory.getSolenoid(NAME, "lock");
        dump = factory.getSolenoid(NAME, "dump");

        coolTimer =
            new AsyncTimer(0.5, () -> lock.set(!needsDump), () -> dump.set(needsDump));
        SmartDashboard.putBoolean("Drive/Overheating", robotState.overheating);
    }

    /** actions */
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

    /** periodic */
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

    /** config and tests */
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

    /** states */
    public enum STATE {
        WAIT,
        DUMP,
    }
}
