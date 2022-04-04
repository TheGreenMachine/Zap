package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.season.Constants;
import edu.wpi.first.util.sendable.SendableBuilder;

@Singleton
public class Shooter extends Subsystem implements PidProvider {

    private static final String NAME = "shooter";

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorControllerEnhanced shooterFollower;
    private final ISolenoid hood;

    // State
    private boolean outputsChanged;

    private boolean hoodOut;
    private double velocityDemand;
    private double actualShooterVelocity;

    // Constants
    private final String pidSlot = "slot0";
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    // we may not need these 4 constants in the near future - still need to move into constructor
    public static final int NEAR_VELOCITY = (int) factory.getConstant(NAME, "nearVel"); // Initiation line
    public static final int MID_VELOCITY = (int) factory.getConstant(NAME, "midVel"); // Trench this also worked from initiation
    public static final int FAR_VELOCITY = (int) factory.getConstant(NAME, "farVel");
    public static final int MAX_VELOCITY = (int) factory.getConstant(NAME, "maxVel");

    public static final int COAST_VELOCITY = (int) factory.getConstant(NAME, "coast");

    // tune this and make changeable with a button in shooter itself
    public final int VELOCITY_THRESHOLD;
    private STATE state = STATE.STOP;

    public Shooter() {
        super(NAME);
        this.shooterMain = factory.getMotor(NAME, "shooterMain");
        this.shooterFollower =
            (IMotorControllerEnhanced) factory.getMotor(
                NAME,
                "shooterFollower",
                shooterMain
            );

        this.shooterFollower.setInverted(true);
        this.hood = factory.getSolenoid(NAME, "hood");

        PIDSlotConfiguration pidConfig = factory.getPidSlotConfig(NAME, pidSlot);

        this.kP = pidConfig.kP;
        this.kI = pidConfig.kI;
        this.kD = pidConfig.kD;
        this.kF = pidConfig.kF;
        VELOCITY_THRESHOLD = pidConfig.allowableError.intValue();

        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterFollower.setNeutralMode(NeutralMode.Coast);

        configCurrentLimits(40/* amps */);

        shooterMain.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(false);
    }

    private void configCurrentLimits(int currentLimitAmps) {
        shooterMain.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0),
            Constants.kCANTimeoutMs
        );
        shooterFollower.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, currentLimitAmps, 0, 0),
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public double getActualVelocity() {
        return actualShooterVelocity;
    }

    public double getTargetVelocity() {
        return velocityDemand;
    }

    public double getError() {
        return Math.abs(actualShooterVelocity - velocityDemand);
    }

    public void setHood(boolean in) {
        hoodOut = in;
        this.outputsChanged = true;
    }

    public void setHood() {
        hoodOut = !hoodOut;
        this.outputsChanged = true;
    }

    public void setVelocity(double velocity) {
        velocityDemand = velocity;
        shooterMain.set(ControlMode.Velocity, velocityDemand);
    }

    public void setDesiredState(STATE state) {
        // no checker for state because we may tell the shooter to set to the same state but different vel
        this.state = state;
        outputsChanged = true;
        System.out.println("desired shooter " + state);
    }

    public boolean isVelocityNearTarget() {
        //        System.out.println("checking if shooter up to speed - " + velocityDemand + " = velocity demand"  + actualShooterVelocity + " = act vel");
        return (
            Math.abs(velocityDemand - actualShooterVelocity) < VELOCITY_THRESHOLD &&
            state != STATE.COASTING
        );
    }

    @Override
    public void readFromHardware() {
        actualShooterVelocity = shooterMain.getSelectedSensorVelocity(0);
        robotState.shooterSpeed =
            convertShooterTicksToMetersPerSecond(actualShooterVelocity);

        if (state != robotState.shooterState) {
            if (actualShooterVelocity < VELOCITY_THRESHOLD) {
                robotState.shooterState = STATE.STOP;
            } else if (isVelocityNearTarget()) {
                robotState.shooterState = STATE.REVVING;
            } else {
                robotState.shooterState = STATE.COASTING;
            }
            if (state == STATE.REVVING) {
                System.out.println(
                    "shooter not at speed! actual state = " + robotState.shooterState
                );
            }
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (state) {
                case STOP:
                    setVelocity(0);
                    break;
                case REVVING: // velocity is set in higher classes (superstructure)
                    break;
                case COASTING:
                    setVelocity(COAST_VELOCITY);
                    break;
            }
            hood.set(hoodOut);
            System.out.println("velocity shooter demand = " + velocityDemand);
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

    private EnhancedMotorChecker.CheckerConfig getTalonCheckerConfig(
        IMotorControllerEnhanced talon
    ) {
        return EnhancedMotorChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    @Override
    public boolean checkSystem() {
        boolean checkShooter = EnhancedMotorChecker.checkMotors(
            this,
            getTalonCheckerConfig(shooterMain),
            new EnhancedMotorChecker.NamedMotor("shooterMain", shooterMain)
        );

        return checkShooter;
    }

    public enum STATE {
        STOP,
        COASTING,
        REVVING,
        CAMERA,
    }
}
