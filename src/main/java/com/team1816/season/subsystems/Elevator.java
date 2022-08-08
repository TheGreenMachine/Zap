package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Elevator extends Subsystem {

    private static final String NAME = "elevator";

    // Components
    private final IGreenMotor elevatorMotor;
    private final DigitalInput ballSensor;

    // State
    private double desiredOutput;
    private double actualOutput;
    private boolean outputsChanged;
    private STATE desiredState = STATE.STOP;

    private final double ALLOWABLE_ERROR;
    private final double FLUSH;
    private final double INTAKE;
    private final double FIRE;
    private final boolean isVelocity;

    public Elevator() {
        super(NAME);
        this.elevatorMotor = factory.getMotor(NAME, "elevator");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);
        this.ballSensor =
            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));

        isVelocity = factory.getConstant(NAME, "isVelocity", 0) > 0;

        // Constants
        double MAX_TICKS = factory.getConstant(NAME, "maxVelTicks100ms", 0);
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

    private void setElevator(double elevatorOutput) {
        if (desiredOutput != elevatorOutput) {
            desiredOutput = elevatorOutput;

            if (isVelocity) {
                elevatorMotor.set(ControlMode.Velocity, elevatorOutput);
            } else {
                elevatorMotor.set(ControlMode.PercentOutput, elevatorOutput);
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
        if (this.desiredState != state) {
            this.desiredState = state;
            outputsChanged = true;
        }
    }

    public double getActualOutput() {
        return actualOutput;
    }

    public double getDesiredOutput() {
        return desiredOutput;
    }

    public boolean hasBallInElevator() {
        return !ballSensor.get();
    }

    @Override
    public void readFromHardware() {
        if (desiredState != robotState.elevatorState) {
            if (isVelocity) {
                actualOutput = elevatorMotor.getSelectedSensorVelocity(0);

                if (Math.abs(actualOutput) < 100) { // TODO make this not a raw number
                    robotState.elevatorState = STATE.STOP;
                } else if (desiredState == STATE.INTAKE) {
                    robotState.elevatorState = desiredState;
                } else if (Math.abs(FIRE - actualOutput) < ALLOWABLE_ERROR) {
                    robotState.elevatorState = STATE.FIRE;
                } else if (desiredOutput < -1000) {
                    robotState.elevatorState = STATE.FLUSH;
                }
            } else {
                robotState.elevatorState = desiredState;
            }
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
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
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        boolean passed = true;
        elevatorMotor.set(ControlMode.PercentOutput, 0.2);
        Timer.delay(1);
        elevatorMotor.set(ControlMode.PercentOutput, -0.2);
        Timer.delay(1);
        elevatorMotor.set(ControlMode.PercentOutput, 0);
        return true;
    }

    public enum STATE {
        STOP,
        INTAKE,
        FIRE,
        FLUSH,
    }
}
