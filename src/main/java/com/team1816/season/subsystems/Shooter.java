package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Subsystem that fires game elements into the hub
 * Used in tandem with DistanceManager in Orchestrator
 * @see DistanceManager
 * @see com.team1816.season.states.Orchestrator
 */
@Singleton
public class Shooter extends Subsystem implements PidProvider {

    /**
     * Properties
     */
    private static final String NAME = "shooter";

    private final PIDSlotConfiguration pidConfig;
    public static final int NEAR_VELOCITY = (int) factory.getConstant(NAME, "nearVel"); // initiation threshold
    public static final int MID_VELOCITY = (int) factory.getConstant(NAME, "midVel");
    public static final int TARMAC_TAPE_VEL = (int) factory.getConstant(
        NAME,
        "tarmacTapeVel"
    );
    public static final int LAUNCHPAD_VEL = (int) factory.getConstant(
        NAME,
        "launchpadVel"
    );
    public static final int MAX_VELOCITY = (int) factory.getConstant(NAME, "maxVel");
    public static final int COAST_VELOCITY = (int) factory.getConstant(NAME, "coast");
    public final int VELOCITY_THRESHOLD;

    /**
     * Components
     */
    private final IGreenMotor shooterMain;

    /**
     * State
     */
    private STATE desiredState = STATE.STOP;
    private boolean outputsChanged;
    private double desiredVelocity;
    private double actualVelocity;

    /**
     * Instantiates a Shooter subsystem with base subsystem methods
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Shooter(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        shooterMain = factory.getMotor(NAME, "shooterMain");
        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterMain.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(false);
        configCurrentLimits(40/* amps */);

        pidConfig = factory.getPidSlotConfig(NAME);
        VELOCITY_THRESHOLD = pidConfig.allowableError.intValue();
    }

    /**
     * Configures current limits
     * @param currentLimitAmps currentLimit
     */
    private void configCurrentLimits(int currentLimitAmps) {
        shooterMain.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0),
            Constants.kCANTimeoutMs
        );
    }

    /**
     * Returns the pid configuration of the motor
     * @return PIDSlotConfiguration
     */
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        return pidConfig;
    }

    /**
     * Returns the actual velocity of the motor
     * @return actualVelocity
     */
    public double getActualVelocity() {
        return actualVelocity;
    }

    /**
     * Returns the desired velocity of the motor
     * @return desiredVelocity
     */
    public double getTargetVelocity() {
        return desiredVelocity;
    }

    /**
     * Returns the error in the actual and desired velocities
     * @return
     */
    public double getError() {
        return Math.abs(actualVelocity - desiredVelocity);
    }

    /** Actions */

    /**
     * Sets the velocity to a demand
     * @param velocity demand
     */
    public void setVelocity(double velocity) {
        desiredVelocity = velocity;
        shooterMain.set(ControlMode.Velocity, desiredVelocity);
    }

    /**
     * Sets the desired state
     * @param state STATE
     */
    public void setDesiredState(STATE state) {
        desiredState = state;
        outputsChanged = true;
    }

    /**
     * Determines if the error is in acceptable margin
     * @return true if error is within threshold
     */
    public boolean isVelocityNearTarget() {
        if (!isImplemented()) {
            return true;
        }
        return (
            Math.abs(desiredVelocity - actualVelocity) < VELOCITY_THRESHOLD &&
            desiredState != STATE.COASTING
        );
    }

    /**
     * Reads actual velocity from the shooterMain motor
     */
    @Override
    public void readFromHardware() {
        actualVelocity = shooterMain.getSelectedSensorVelocity(0);

        if (desiredState != robotState.shooterState) {
            if (actualVelocity < VELOCITY_THRESHOLD) {
                robotState.shooterState = STATE.STOP;
            } else if (isVelocityNearTarget()) {
                robotState.shooterState = STATE.REVVING;
            } else {
                robotState.shooterState = STATE.COASTING;
            }
        }
    }

    /**
     * Sets velocity of the shooter based on the desired state
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP:
                    setVelocity(0);
                    break;
                case REVVING: // velocity is set in orchestrator
                    break;
                case COASTING:
                    setVelocity(COAST_VELOCITY);
                    break;
            }
        }
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {}

    /** Config and Tests */

    /**
     * Initializes a SendableBuilder for SmartDashboard
     * @param builder SendableBuilder
     */
    @Override
    public void initSendable(SendableBuilder builder) {}

    /**
     * Stops the shooter
     */
    @Override
    public void stop() {}

    /**
     * Tests the subsystem
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        boolean checkShooter = EnhancedMotorChecker.checkMotor(this, shooterMain);

        return checkShooter;
    }

    /**
     * Base enum for Shooter subsystem state
     */
    public enum STATE {
        STOP,
        COASTING,
        REVVING,
        CAMERA,
    }
}
