package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

@Singleton
public class Hopper extends Subsystem {
    // from what I understand, we want to make the elevator use velocity control
    // and also be in tandem with the shooter (w/ distance manager) so that depending on the shot distance,
    // the elevator will be given a wee bit more or less umph

    private static final String NAME = "hopper";

    // Components
    private final ISolenoid feederFlap;
    private final IMotorControllerEnhanced spindexer;
    private final IMotorControllerEnhanced elevator;

    @Inject
    private static Shooter shooter;

    @Inject
    private static DistanceManager distanceManager;

    @Inject
    private static Camera camera;

    private final DigitalInput ballSensor;

    // State
    private boolean feederFlapOut;
    private double spindexerPower;
    private double elevatorPower;
    private boolean outputsChanged;

    private boolean lockToShooter;
    private int waitForShooterLoopCounter;
    private boolean shooterWasAtTarget;

    private boolean wantUnjam;

    public Hopper() {
        super(NAME);
        this.feederFlap = factory.getSolenoid(NAME, "feederFlap");
        this.spindexer = factory.getMotor(NAME, "spindexer");
        this.elevator = factory.getMotor(NAME, "elevator");
        this.ballSensor =
            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));
    }

    public void setFeederFlap(boolean feederFlapOut) {
        this.feederFlapOut = feederFlapOut;
        outputsChanged = true;
    }

    public void setSpindexer(double spindexerOutput) {
        this.spindexerPower = 0.25 * spindexerOutput;
        outputsChanged = true;
    }

    public void startSpindexerBasedOnDistance() {
        setSpindexer(distanceManager.getSpindexerOutput(0));
    }

    public void setElevator(double elevatorOutput) {
        this.elevatorPower = elevatorOutput;
        outputsChanged = true;
    }

    public void setIntake(double intakeOutput) {
        setElevator(intakeOutput);
        if (intakeOutput > 0) {
            setSpindexer(intakeOutput);
//            startSpindexerBasedOnDistance(); INTRODUCE DISTANCEMANAGER
        } else {
            setSpindexer(0);
        }
    }

    public void lockToShooter(boolean lock, boolean unjam) {
        this.lockToShooter = lock;
        this.wantUnjam = unjam;
        this.waitForShooterLoopCounter = 0;
    }

    public boolean hasBallInElevator() {
        return ballSensor.get();
    }
    // TODO: implement when we have color sensors
    public boolean colorOfBall() { return false; }

    /**
     * Should the shooter fire immediately, 
     * or should it pause and reverse the spindexer?
     */
    public boolean shouldFire() {
        if (wantUnjam) return false;
        boolean fire = false;
        if (hasBallInElevator()) { // color sensor check for team color here
            fire = shooter.isVelocityNearTarget();
        }
        return fire;
    }

    @Override
    public void writeToHardware() {
        if (lockToShooter) {
            if (!shooter.isVelocityNearTarget()) {
                setSpindexer(-1);
            } else {
               setSpindexer(1);
            }
            System.out.println("Near Velocity: " + shooter.isVelocityNearTarget());
            // System.out.println("Has Ball: " + hasBallInElevator());
            if (!shouldFire()) {
                // spindexer needs to be stopped so that it doesn't accidentally
                // push the ball back in / etc
                this.spindexer.set(ControlMode.PercentOutput, 0);
                // spit the ball back out
                this.elevator.set(ControlMode.PercentOutput, -0.25);
                return;
            }
            // 
            lockToShooter = false;
            shooterWasAtTarget = true;
            setSpindexer(1);
            setElevator(1);
        }
        if (outputsChanged) {
            this.spindexer.set(ControlMode.PercentOutput, spindexerPower);
            this.elevator.set(ControlMode.PercentOutput, elevatorPower);
            this.feederFlap.set(feederFlapOut);
            outputsChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Hopper/HasBall", this::hasBallInElevator, null);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
