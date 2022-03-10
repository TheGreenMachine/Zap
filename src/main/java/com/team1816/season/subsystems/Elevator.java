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

    public void autoElevator(double elevatorOutput) {
        distanceManaged = true;
        this.elevatorPower = elevatorOutput;
    }

    public void setState(ELEVATOR_STATE state) {
        if(this.state != state){
            this.state = state;
            System.out.println("ELEVATOR STATE IS CHANGED TO " + state);
            outputsChanged = true;
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
    public void writeToHardware() {
        if (outputsChanged) {
            double pow = 0;
            switch (state) {
                case STOP:
                    pow = 0;
                    break;
                case FIRE:
                    if (!distanceManaged){
                        pow = FIRE;
                    } else {
                        pow = elevatorPower;
                    }
                    break;
                case FLUSH:
                    pow = FLUSH;
                    break;
            }
            this.elevator.set(ControlMode.PercentOutput, pow);
            // create ball color updating here once sensor created
            distanceManaged = false;
            outputsChanged = false;
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
