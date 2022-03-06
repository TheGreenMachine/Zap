package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
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
    private boolean distanceManaged = false;

    private boolean hoodOut;
    private double velocityDemand;
    private double actualShooterVelocity;
    private double closedLoopError;

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
    private static final int DEFAULT_REV_VELOCITY = (int) factory.getConstant(NAME, "defaultRevVel", MID_VELOCITY);


    // tune this and make changeable with a button in shooter itself
    public static final int VELOCITY_THRESHOLD = (int) factory.getConstant(
        NAME,
        "velocityThreshold",
        500
    );
    private SHOOTER_STATE state = SHOOTER_STATE.STOP;

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

    public void setVelocity(double velocity) {
        velocityDemand = velocity;
        distanceManaged = true;
        outputsChanged = true;
    }

    public void setHood(boolean in) {
        hoodOut = in;
        this.outputsChanged = true;
    }

    public void setState(SHOOTER_STATE state) {
        this.state = state;
        this.outputsChanged = true;
    }

    public boolean isVelocityNearTarget() {
        return (
            Math.abs(closedLoopError) < VELOCITY_THRESHOLD &&
                (int) velocityDemand != COAST_VELOCITY
        );
    }

    @Override
    public void readFromHardware() {
        actualShooterVelocity = shooterMain.getSelectedSensorVelocity(0);
        closedLoopError = shooterMain.getClosedLoopError(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            switch (state){
                case STOP:
                    velocityDemand = 0;
                    break;
                case REVVING:
                    if (!distanceManaged) velocityDemand = DEFAULT_REV_VELOCITY;
                    break;
                case COASTING:
                    velocityDemand = COAST_VELOCITY;
                    break;
            }
            hood.set(hoodOut);
            shooterMain.set(ControlMode.Velocity, velocityDemand);

            System.out.println("velocity shooter demand = " + velocityDemand);
            outputsChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Shooter/IsAtSpeed", this::isVelocityNearTarget, null);
        builder.addDoubleProperty(
            "Shooter/ShooterVelocity",
            this::getActualVelocity,
            this::setVelocity
        );
    }

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

    public static class PeriodicIO {

        //INPUTS
        public double actualShooterVelocity;
        public double closedLoopError;

        //OUPUTS
        public double velocityDemand;
    }

    public enum SHOOTER_STATE {
        STOP,
        COASTING,
        REVVING,
    }
}
