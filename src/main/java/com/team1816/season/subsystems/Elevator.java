package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Inject;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

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


    public Elevator() {
        super(NAME);
        this.elevator = factory.getMotor(NAME, "elevator");
//        this.ballSensor =
//            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));
    }

    public void autoElevator(double elevatorOutput) {
        distanceManaged = true;
        this.elevatorPower = elevatorOutput;
    }

    public void setState(ELEVATOR_STATE state){
        this.state = state;
        outputsChanged = true;
    }

    public boolean hasBallInElevator() {
        return false;
//        return ballSensor.get(); // TODO get Digital IO
    }

    // TODO: implement when we have color sensors
    public boolean colorOfBall() { return true; }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            switch (state){
                case STOP:
                    elevatorPower = 0;
                case FIRING:
                    if(!distanceManaged)
                    elevatorPower = 1;
                case FLUSH:
                    elevatorPower = -1;
            }
            this.elevator.set(ControlMode.PercentOutput, elevatorPower);
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
        FIRING,
        FLUSH,
    }
}
