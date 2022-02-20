package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.EnhancedMotorChecker;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class Shooter extends Subsystem implements PidProvider {

    private static final String NAME = "shooter";

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorControllerEnhanced shooterFollower;

    @Inject
    private static LedManager ledManager;

    private final ISolenoid hood;
    // State
    private boolean outputsChanged;
    private boolean hoodOut = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // Constants
    private final String pidSlot = "slot0";
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    public static final int MAX_VELOCITY = 11_800; // Far
    public static final int NEAR_VELOCITY = 11_100; // Initiation line
    public static final int MID_VELOCITY = 9_900; // Trench this also worked from initiation
    public static final int MID_FAR_VELOCITY = 11_200;
    public static final int VELOCITY_THRESHOLD = (int) factory.getConstant(
        NAME,
        "velocityThreshold",
        3000
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

    public void setVelocity(double velocity) {
        mPeriodicIO.velocityDemand = velocity;
        outputsChanged = true;
    }

    public boolean isHoodOut() {
        return hoodOut;
    }

    public void setHood(boolean in) {
        hoodOut = in;
        this.outputsChanged = true;
    }

    public void setState(SHOOTER_STATE state){
        this.state = state;
    }

    public double getActualVelocity() {
        return mPeriodicIO.actualShooterVelocity;
    }

    public double getTargetVelocity() {
        return mPeriodicIO.velocityDemand;
    }

    public double getError() {
        return mPeriodicIO.closedLoopError;
    }

    public boolean isVelocityNearTarget() {
        return Math.abs(this.getError()) < VELOCITY_THRESHOLD;
    }

    @Override
    public void readFromHardware() {
        mPeriodicIO.actualShooterVelocity = shooterMain.getSelectedSensorVelocity(0);
        mPeriodicIO.closedLoopError = shooterMain.getClosedLoopError(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            this.hood.set(hoodOut);
            if (mPeriodicIO.velocityDemand == 0) {
                this.shooterMain.set(ControlMode.PercentOutput, 0); // Inertia coast ! Set 0 to coasting value
            } else {
                this.shooterMain.set(ControlMode.Velocity, mPeriodicIO.velocityDemand);
            }
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

    public enum SHOOTER_STATE{
        STOP,
        COASTING,
        REVVING
    }
}
