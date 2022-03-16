package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

import static com.ctre.phoenix.motorcontrol.ControlMode.Position;

public class Climber extends Subsystem {

    // this class uses some logic from the turret (positionControl logic) and from the distanceManager (Stage behaves like bucket Entries)
    private static final String NAME = "climber";

    // Components
    private final IMotorControllerEnhanced elevator;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    // State
    private ControlMode controlMode = ControlMode.MANUAL;
    // Position
    private int currentStage;
    private final Stage[] stages;
    private double climberPosition;
    //Manual
    private double climberPower;
    private boolean topClamped;
    private boolean bottomClamped;
    private boolean outputsChanged = false;

    private final double ALLOWABLE_ERROR = factory.getConstant(NAME, "allowableError", 50);
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

        currentStage = 0;

        stages = new Stage[]{
            new Stage(factory.getConstant(NAME, "startPos", 0), false, true),
            new Stage(factory.getConstant(NAME, "firstToSecondRungPos", -200), true, false),
            new Stage(factory.getConstant(NAME, "secondToLastRungPos", -200), false, true),
            new Stage(factory.getConstant(NAME, "lastPos", -200), true, false)
        };
    }

    public void incrementClimberStage(){ // we can't go backwards (descend rungs) using this logic, but it shouldn't really matter
        if (controlMode != ControlMode.POSITION) {
            controlMode = ControlMode.POSITION;
        }

        if(Math.abs(elevator.getSelectedSensorPosition(0) - stages[currentStage].position) < ALLOWABLE_ERROR){
            currentStage++;
            outputsChanged = true;
        } else {
            System.out.println("climber not safely at stage " + currentStage + " - not incrementing stage!");
        }
    }

    public void setClimberPower(double power){
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        climberPosition = power;
        outputsChanged = true;
    }

    public void setTopClamp(boolean topClamped) {
        if(controlMode == ControlMode.MANUAL){
            this.topClamped = topClamped;
        } else {
            System.out.println("climber not in manual mode! - not clamping");
        }
        outputsChanged = true;
    }

    public void setBottomClamp(boolean bottomClamped) {
        if(controlMode == ControlMode.MANUAL){
            this.bottomClamped = bottomClamped;
        } else {
            System.out.println("climber not in manual mode! - not clamping");
        }
        outputsChanged = true;
    }

//    public void setClimberAngle(double angle) {
//        setClimberPosition(convertToTurretTicks(angle));
//    }

    private void positionControl(double position) { // if we ever need to do offsets, do them here
        elevator.set(Position, position);
    }

    private void setClamps(boolean topClamped, boolean bottomClamped) {
        if(topClamp.get() != topClamped){
            topClamp.set(topClamped);
        }
        if(bottomClamp.get() != bottomClamped){
            bottomClamp.set(bottomClamped);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            if (controlMode == ControlMode.POSITION) {
                positionControl(stages[currentStage].position);
                setClamps(stages[currentStage].topClamped, stages[currentStage].bottomClamped);
            } else {
                elevator.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, climberPower);
                setClamps(topClamped, bottomClamped);
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

    public enum ControlMode {
        MANUAL,
        POSITION,
    }

    static class Stage {

        public final double position;
        public final boolean topClamped;
        public final boolean bottomClamped;

        Stage(
            double distance,
            boolean topClamped,
            boolean bottomClamped
        ) {
            this.position = distance;
            this.topClamped = topClamped;
            this.bottomClamped = bottomClamped;
        }

        Stage() {
            this(0, false, false);
        }
    }
}
