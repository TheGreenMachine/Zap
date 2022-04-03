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
    private double elevatorOutput;
    private boolean outputsChanged;
    private ELEVATOR_STATE state = ELEVATOR_STATE.STOP;


    // Constants
    private final double MAX_TICKS;
    private final double FLUSH;
    private double FIRE; // bear in mind this is overridden
    private final boolean isVelocity;

    public Elevator() {
        super(NAME);
        this.elevator = factory.getMotor(NAME, "elevator");

        //        this.ballSensor =
        //            new DigitalInput((int) factory.getConstant(NAME, "ballSensor", 0));

        isVelocity = factory.getConstant(NAME, "isVelocity", 0) > 0;

        MAX_TICKS = factory.getConstant(NAME, "maxTicks", 0);
        if(!isVelocity){
            FLUSH = factory.getConstant(NAME, "flushPow", -0.5);
            FIRE = factory.getConstant(NAME, "firePow", 0.5);
        } else {
            FLUSH = factory.getConstant(NAME, "flushPow", -0.5) * MAX_TICKS;
            FIRE = factory.getConstant(NAME, "firePow", 0.5) * MAX_TICKS;
        }
    }

    public void overridePower(double newFirePow){
        FIRE = newFirePow;
    }

    private void setElevator(double elevatorOutput) {
        this.elevatorOutput = elevatorOutput;

        if(isVelocity) {
            System.out.println(elevatorOutput + " = elevator velocity");
            this.elevator.set(ControlMode.Velocity, elevatorOutput);
        } else {
            System.out.println(elevatorOutput + " = elevator power");
            this.elevator.set(ControlMode.PercentOutput, elevatorOutput);
        }
    }

    private void lockToShooter(){
        if (robotState.shooterState == Shooter.SHOOTER_STATE.REVVING) {
            setElevator(FIRE);
        } else {
            outputsChanged = true; // keep looping through writeToHardware if shooter not up to speed
        }
    }

    public void setDesiredState(ELEVATOR_STATE state) {
        if (this.state != state) {
            this.state = state;
            outputsChanged = true;
            System.out.println("desired elevator " + state);
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
//        double actualVel = elevator.getSelectedSensorVelocity(0);
        if(state != robotState.elevatorState){
//            if(Math.abs(elevatorPower) == 0) {
//                robotState.elevatorState = ELEVATOR_STATE.STOP;
//            } else if (elevatorPower > POWER_THRESHOLD) {
//                robotState.elevatorState = ELEVATOR_STATE.FIRE;
//            } else if(elevatorPower < -POWER_THRESHOLD){
//                robotState.elevatorState = ELEVATOR_STATE.FLUSH;
//            }
            robotState.elevatorState = state;

            System.out.println("ACTUAL ELEVATOR STATE = " + robotState.elevatorState);
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
        return true;
    }

    public enum ELEVATOR_STATE {
        STOP,
        FIRE,
        FLUSH,
    }
}
