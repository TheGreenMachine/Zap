package com.team1816.lib.subsystems;

import badlog.lib.BadLog;
import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.ILooper;
import com.team1816.season.Robot;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.function.Supplier;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * initializing all member components at the enableDigital of the match.
 */
public abstract class Subsystem implements Sendable {

    private final String name;
    public static RobotFactory factory = Robot.getFactory();

    @Inject
    public static RobotState robotState;

    @Inject
    public static Infrastructure infrastructure;

    protected Subsystem(String name) {
        this.name = name;
        SendableRegistry.addLW(this, name, name);
    }

    public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readFromHardware() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writeToHardware() {}

    public void registerEnabledLoops(ILooper mEnabledLooper) {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public void CreateBadLogTopic(
        String topicName,
        String unit,
        Supplier<Double> supplier,
        String... attrs
    ) {
        if (factory.getSubsystem(name).implemented) {
            BadLog.createTopic(topicName, unit, supplier, attrs);
        }
    }

    public void CreateBadLogValue(String badLogName, String value) {
        if (factory.getSubsystem(name).implemented) {
            BadLog.createValue(badLogName, value);
        }
    }

    @Deprecated
    public void outputTelemetry() {}

    @Override
    public void initSendable(SendableBuilder builder) {}

    public String getSubsystemName() {
        return name;
    }

    public boolean isImplemented() {
        return factory.getSubsystem(name).implemented;
    }
}
