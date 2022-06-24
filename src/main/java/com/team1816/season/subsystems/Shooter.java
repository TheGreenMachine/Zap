package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.season.Constants;
import edu.wpi.first.util.sendable.SendableBuilder;

@Singleton
public class Shooter extends Subsystem implements PidProvider {

    private static final String NAME = "shooter";

    // Components
    private final IGreenMotor shooterMain;

    // State
    private STATE desiredState = STATE.STOP;
    private boolean outputsChanged;
    private double desiredVelocity;
    private double actualVelocity;

    // Constants
    private final PIDSlotConfiguration pidConfig;
    // we may not need these 4 constants in the near future - still need to move into constructor
    public static final int NEAR_VELOCITY = (int) factory.getConstant(NAME, "nearVel"); // Initiation line
    public static final int MID_VELOCITY = (int) factory.getConstant(NAME, "midVel"); // Trench this also worked from initiation
    public static final int TARMAC_TAPE_VEL = (int) factory.getConstant(
        NAME,
        "tarmacTapeVel"
    ); // Trench this also worked from initiation
    public static final int LAUNCHPAD_VEL = (int) factory.getConstant(
        NAME,
        "launchpadVel"
    );
    public static final int MAX_VELOCITY = (int) factory.getConstant(NAME, "maxVel");

    public static final int COAST_VELOCITY = (int) factory.getConstant(NAME, "coast");

    // tune this and make changeable with a button in shooter itself
    public final int VELOCITY_THRESHOLD;

    public Shooter() {
        super(NAME);
        shooterMain = factory.getMotor(NAME, "shooterMain");
        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterMain.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(false);
        configCurrentLimits(40/* amps */);

        pidConfig = factory.getPidSlotConfig(NAME);
        VELOCITY_THRESHOLD = pidConfig.allowableError.intValue();
    }

    private void configCurrentLimits(int currentLimitAmps) {
        shooterMain.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0),
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        return pidConfig;
    }

    public double getActualVelocity() {
        return actualVelocity;
    }

    public double getTargetVelocity() {
        return desiredVelocity;
    }

    public double getError() {
        return Math.abs(actualVelocity - desiredVelocity);
    }

    public void setVelocity(double velocity) {
        desiredVelocity = velocity;
        shooterMain.set(ControlMode.Velocity, desiredVelocity);
    }

    public void setDesiredState(STATE state) {
        // no checker for state because we may tell the shooter to set to the same state but different vel
        desiredState = state;
        outputsChanged = true;
    }

    public boolean isVelocityNearTarget() {
        if (!isImplemented()) {
            /*
            this is here to let us use rampUpShooterAction or getShoot on a robot that doesn't have a shooter without
            having the robot freeze up (ie: a path won't continue until the shooter has finished ramping up)
            */
            return true;
        }
        return (
            Math.abs(desiredVelocity - actualVelocity) < VELOCITY_THRESHOLD &&
            desiredState != STATE.COASTING
        );
    }

    @Override
    public void readFromHardware() {
        actualVelocity = shooterMain.getSelectedSensorVelocity(0);

        robotState.shooterMPS = convertShooterTicksToMetersPerSecond(actualVelocity);

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

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP:
                    setVelocity(0);
                    break;
                case REVVING: // velocity is set in higher classes (superstructure)
                    break;
                case COASTING:
                    setVelocity(COAST_VELOCITY);
                    break;
            }
        }
    }

    public double convertShooterTicksToMetersPerSecond(double ticks) {
        return 0.00019527 * ticks; //TODO: verify conversion (roughly accurate based on recorded data)
    }

    public double convertShooterMetersToTicksPerSecond(double metersPerSecond) {
        return (metersPerSecond + 0.53) / 0.0248;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        boolean checkShooter = EnhancedMotorChecker.checkMotor(this, shooterMain);

        return checkShooter;
    }

    public enum STATE {
        STOP,
        COASTING,
        REVVING,
        CAMERA,
    }
}
