package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

@Singleton
public class Elevator extends Subsystem {

    private static final String NAME = "elevator";

    // Components
    private final IMotorControllerEnhanced elevator;
    //    private final DigitalInput ballSensor;

    // State
    private double elevatorPower;
    private boolean outputsChanged;
    private ELEVATOR_STATE state = ELEVATOR_STATE.STOP;
    private boolean distanceManaged = false;

    // Constants
    private final double FLUSH;
    private final double FIRE;

    public Elevator() {
        super(NAME);
        this.elevator = factory.getMotor(NAME, "elevator");

        //        this.ballSensor =
        //            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));

        FLUSH = factory.getConstant(NAME, "flushPow", -0.5);
        FIRE = factory.getConstant(NAME, "firePow", 0.5);
    }

    public void setElevator(double elevatorOutput) {
        this.elevatorPower = elevatorOutput;
        outputsChanged = true;
    }

    public void setDesiredState(ELEVATOR_STATE state) {
        if (this.state != state) {
            this.state = state;
            switch (state) {
                case STOP:
                    setElevator(0);
                    robotState.elevatorState = ELEVATOR_STATE.STOP;
                    break;
                case FIRE: // dealt with in read - waiting for shooter state
                    break;
                case FLUSH:
                    setElevator(FLUSH);
                    robotState.elevatorState = ELEVATOR_STATE.FLUSH;
                    break;
            }
            System.out.println("DESIRED ELEVATOR STATE = " + state);
        }
    }

    public boolean hasBallInElevator() {
        return false;
        //        return ballSensor.get(); // TODO get Digital IO
    }

    // TODO: implement when we have color sensors
    public boolean colorOfBall() {
        return true;
    }

    @Override
    public void readFromHardware() {
        if (
            robotState.shooterState == Shooter.SHOOTER_STATE.REVVING &&
            state != robotState.elevatorState
        ) {
            robotState.elevatorState = ELEVATOR_STATE.FIRE;
            setElevator(FIRE);
            if (elevatorPower != FIRE) {
                System.out.println("ACTUAL ELEVATOR STATE = FIRE");
            }
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;

            System.out.println(elevatorPower + " = elevator power");
            this.elevator.set(ControlMode.PercentOutput, elevatorPower);
            // create ball color updating here once sensor created
            distanceManaged = false;
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
        return true;
    }

    public enum ELEVATOR_STATE {
        STOP,
        FIRE,
        FLUSH,
    }
}
