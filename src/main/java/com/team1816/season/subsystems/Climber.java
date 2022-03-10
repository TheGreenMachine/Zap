package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

public class Climber extends Subsystem {

    // climber this year has 2 motors: one for going up/down, and one for making the ferris wheel "spin". 4? pistons,
    // one for each claw, (two will be fired at once) - If you have no idea what the climber does or looks like,
    // ask max or nico or, even better, bully (ask) the build team because they better know about what they're building!

    private static final String NAME = "climber";

    // Components
    private final IMotorControllerEnhanced elevator;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    // State
    private STATE state = STATE.MANUAL;
    private double climberPower;
    private double climberPosition;
    private boolean isDeployed;
    private boolean outputsChanged = false;

    private final double MAX_POSITION = factory.getConstant("climberMaxPosition", -200);
    private final double MIN_POSITION = factory.getConstant("climberMinPosition", 0);
    private final double MID_POSITTION = (MAX_POSITION + MIN_POSITION)/2;


    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        topClamp = factory.getSolenoid(NAME, "topClamp");
        bottomClamp = factory.getSolenoid(NAME, "bottomClamp");
    }

    public void setClimberPower(double power) {
        if(state != STATE.MANUAL){
            state = STATE.MANUAL;
        }
        climberPower = power;
        outputsChanged = true;
    }

    public void setClimberUp() {
        if(state != STATE.POSITION){
            state = STATE.POSITION;
        }
        climberPosition = MAX_POSITION;
    }

    public void setClimberMid() {
        if(state != STATE.POSITION){
            state = STATE.POSITION;
        }
        climberPosition = MID_POSITTION;    }

    public void setClimberDown() {
        if(state != STATE.POSITION){
            state = STATE.POSITION;
        }
        climberPosition = MIN_POSITION;    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            if(state == STATE.POSITION){
                elevator.set(ControlMode.Position, climberPosition);
            } else {
                elevator.set(ControlMode.PercentOutput, climberPower);
            }
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    public enum STATE{
        MANUAL,
        POSITION
    }
}
