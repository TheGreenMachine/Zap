package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

public class Climber extends Subsystem {

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

    // maybe make the climber positions into an array that you go up as you traverse each rung - each array index corresponds to a diff behavior
    // (aka index 0 = unclamped bottom clamps + set motor to position 0, index 1 = clamped top + set motor to 1st position, etc...)
    private final double MAX_POSITION = factory.getConstant(NAME, "climberMaxPos", -200);
    private final double MIN_POSITION = factory.getConstant(NAME, "climberMinPos", 0);
    private final double MID_POSITTION = (MAX_POSITION + MIN_POSITION) / 2;
    private final String pidSlot = "slot0";

    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        topClamp = factory.getSolenoid(NAME, "topClamp");
        bottomClamp = factory.getSolenoid(NAME, "bottomClamp");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME, pidSlot);

        elevator.config_kP(0, config.kP, 100);
        elevator.config_kI(0, config.kI, 100);
        elevator.config_kD(0, config.kD, 100);
        elevator.config_kF(0, config.kF, 100);
    }

    public void setClimberPower(double power) {
        if (state != STATE.MANUAL) {
            state = STATE.MANUAL;
        }
        climberPower = power;
        outputsChanged = true;
    }

    public void setClimberUp() { // make this into method traversing the index?
        if (state != STATE.POSITION) {
            state = STATE.POSITION;
        }
        climberPosition = MAX_POSITION;
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            if (state == STATE.POSITION) {
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

    public enum STATE {
        MANUAL,
        POSITION,
    }
}
