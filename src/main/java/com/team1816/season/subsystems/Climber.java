package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.subsystems.Subsystem;

public class Climber extends Subsystem {

    // climber this year has 2 motors: one for going up/down, and one for making the ferris wheel "spin". 4? pistons,
    // one for each claw, (two will be fired at once) - If you have no idea what the climber does or looks like,
    // ask max or nico or, even better, bully (ask) the build team because they better know about what they're building!

    private static final String NAME = "climber";

    // Components
    //    private final IMotorControllerEnhanced elevator;
    private final IMotorControllerEnhanced elevator;

    // State
    private double climberPow;
    private boolean isDeployed;
    private boolean outputsChanged = false;

//    private double climberMaxPosition = factory.getConstant("climberMaxPosition", -200);
//    private double climberMinPosition = factory.getConstant("climberMinPosition", -20);
//    private double climberMidPosition = (climberMaxPosition + climberMinPosition)/2;


    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        //elevator = new CANSparkMax( 20, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    public void setClimberPower(double power) {
        climberPow = power;
        outputsChanged = true;
    }

//    public void setClimberPosition(double position) {
//        elevator.set(ControlMode.Position, position);
//    }
//
//    public void setClimberUp() {
//        elevator.set(ControlMode.Position, climberMaxPosition);
//    }
//
//    public void setClimberMid() {
//        elevator.set(ControlMode.Position, climberMidPosition);
//    }
//
//    public void setClimberDown() {
//        elevator.set(ControlMode.Position, climberMinPosition);
//    }

    public boolean getDeployed() {
        return isDeployed;
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            //            elevator.set(ControlMode.PercentOutput, climberPow);
            elevator.set(ControlMode.PercentOutput, climberPow);
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
