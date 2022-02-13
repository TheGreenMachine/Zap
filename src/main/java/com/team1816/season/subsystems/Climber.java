package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

public class Climber extends Subsystem {

    private static final String NAME = "climber";

    // Components
    private final IMotorControllerEnhanced elevator;
    private final ISolenoid deployer;

    // State
    private double climberPow;
    private boolean isDeployed;
    private boolean outputsChanged = false;

    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        deployer = factory.getSolenoid(NAME, "deployer");
    }

    public void setClimberPower(double power) {
        climberPow = power;
        outputsChanged = true;
    }

    public void setDeployed(boolean deployed) {
        this.isDeployed = deployed;
        outputsChanged = true;
    }

    public boolean getDeployed(){
        return isDeployed;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            elevator.set(ControlMode.PercentOutput, climberPow);
            deployer.set(isDeployed);
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
