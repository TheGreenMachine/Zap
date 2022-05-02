package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Elevator extends Subsystem {

    private static final String NAME = "elevator";

    // Components
    private final IMotorControllerEnhanced elevator;
    private final DigitalInput ballSensor;

    // State
    private double elevatorOutput;
    private boolean outputsChanged;
    private double actualOutput;
    private STATE state = STATE.STOP;

    // Constants
    private final double MAX_TICKS;
    private final double ALLOWABLE_ERROR;
    private final double FLUSH;
    private final double INTAKE;
    private double FIRE; // bear in mind this is overridden
    private final boolean isVelocity;
    private final String pidSlot = "slot0";

    public Elevator() {
        super(NAME);
        this.elevator = factory.getMotor(NAME, "elevator");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME, pidSlot);
        this.ballSensor =
            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));

        isVelocity = factory.getConstant(NAME, "isVelocity", 0) > 0;

        MAX_TICKS = factory.getConstant(NAME, "maxTicks", 0);
        if (!isVelocity) {
            FLUSH = factory.getConstant(NAME, "flushPow", -0.5);
            FIRE = factory.getConstant(NAME, "firePow", 0.5);
            INTAKE = factory.getConstant(NAME, "intakePow", 0.05);
        } else {
            FLUSH = factory.getConstant(NAME, "flushPow", -0.5) * MAX_TICKS;
            FIRE = factory.getConstant(NAME, "firePow", 0.5) * MAX_TICKS;
            INTAKE = factory.getConstant(NAME, "intakePow", 0.05) * MAX_TICKS;
        }
        ALLOWABLE_ERROR = config.allowableError;
    }

    public void overridePower(double newFirePow) {
        FIRE = newFirePow;
    }

    private void setElevator(double elevatorOutput) {
        if (this.elevatorOutput != elevatorOutput) {
            this.elevatorOutput = elevatorOutput;

            if (isVelocity) {
                //                System.out.println("elevator velocity: " + elevatorOutput);
                this.elevator.set(ControlMode.Velocity, elevatorOutput);
            } else {
                //                System.out.println("elevator power: " + elevatorOutput);
                this.elevator.set(ControlMode.PercentOutput, elevatorOutput);
            }
        }
    }

    private void lockToShooter() {
        if (robotState.shooterState == Shooter.STATE.REVVING) {
            setElevator(FIRE);
        } else {
            outputsChanged = true; // keep looping through writeToHardware if shooter not up to speed
        }
    }

    private void lockToSensor() {
        if (hasBallInElevator()) {
            setDesiredState(STATE.STOP);
        } else {
            setElevator(INTAKE);
            outputsChanged = true; // keep looping through writeToHardware if no ball seen
        }
    }

    public void setDesiredState(STATE state) {
        if (this.state != state) {
            this.state = state;
            outputsChanged = true;
            System.out.println("desired elevator changed: " + state);
        }
    }

    public double getActualOutput() {
        return actualOutput;
    }

    public double getDesiredOutput() {
        return elevatorOutput;
    }

    public boolean hasBallInElevator() {
        return !ballSensor.get();
    }

    // TODO: implement when we have color sensors
    public boolean colorOfBall() {
        return true;
    }

    @Override
    public void readFromHardware() {
        if (state != robotState.elevatorState) {
            if (isVelocity) {
                actualOutput = elevator.getSelectedSensorVelocity(0);
                if (Math.abs(actualOutput) < 100) { // TODO make this not a raw number
                    robotState.elevatorState = STATE.STOP;
                } else if (state == STATE.INTAKE) {
                    robotState.elevatorState = state;
                } else if (Math.abs(FIRE - actualOutput) < ALLOWABLE_ERROR) {
                    robotState.elevatorState = STATE.FIRE;
                } else if (elevatorOutput < -1000) {
                    robotState.elevatorState = STATE.FLUSH;
                }
            } else {
                robotState.elevatorState = state;
            }

            System.out.println(
                "desired elevator: " +
                state +
                ", actual state: " +
                robotState.elevatorState
            );
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (state) {
                case STOP:
                    setElevator(0);
                    break;
                case INTAKE:
                    lockToSensor();
                    break;
                case FIRE:
                    lockToShooter();
                    break;
                case FLUSH:
                    setElevator(FLUSH);
                    break;
            }
            // create ball color updating here once sensor created
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //        builder.addBooleanProperty("Hopper/HasBall", this::hasBallInElevator, null);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        boolean passed = true;
        elevator.set(ControlMode.PercentOutput, 0.2);
        Timer.delay(1);
        elevator.set(ControlMode.PercentOutput, -0.2);
        Timer.delay(1);
        elevator.set(ControlMode.PercentOutput, 0);
        return true;
    }

    public enum STATE {
        STOP,
        INTAKE,
        FIRE,
        FLUSH,
    }
}
