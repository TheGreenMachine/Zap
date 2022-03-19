package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.Timer;

import javax.swing.*;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.ControlMode.Position;

public class Climber extends Subsystem {

    // this class uses some logic from the turret (positionControl logic) and from the distanceManager (Stage behaves like bucket Entries)
    private static final String NAME = "climber";

    // Components
    private final IMotorControllerEnhanced elevator;
    private final IMotorControllerEnhanced elevatorFollower;
    private final ISolenoid topClamp;
    private final ISolenoid bottomClamp;

    // State
    private ControlMode controlMode = ControlMode.MANUAL;
    private double error;
    private boolean needsOverShoot = false;
    private boolean needsClamp = false;
    // Position
    private int currentStage;
    private final Stage[] stages;
    private double climberPosition;
    //Manual
    private double climberPower = 0;
    private boolean topClamped = false;
    private boolean bottomClamped = false;
    private boolean outputsChanged = false;

    private final double ALLOWABLE_ERROR;
    private final String pidSlot = "slot0";

    public Climber() {
        super(NAME);
        elevator = factory.getMotor(NAME, "elevator");
        elevatorFollower = (IMotorControllerEnhanced) factory.getMotor(NAME, "elevatorFollower", elevator, true);
        topClamp = factory.getSolenoid(NAME, "topClamp");
        bottomClamp = factory.getSolenoid(NAME, "bottomClamp");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME, pidSlot);

        ALLOWABLE_ERROR = config.allowableError;

        elevator.config_kP(0, config.kP, 100);
        elevator.config_kI(0, config.kI, 100);
        elevator.config_kD(0, config.kD, 100);
        elevator.config_kF(0, config.kF, 100);

        elevatorFollower.config_kP(0, config.kP, 100);
        elevatorFollower.config_kI(0, config.kI, 100);
        elevatorFollower.config_kD(0, config.kD, 100);
        elevatorFollower.config_kF(0, config.kF, 100);

        elevator.configClosedloopRamp(.5, Constants.kCANTimeoutMs);
        elevatorFollower.configClosedloopRamp(.5, Constants.kCANTimeoutMs);

        currentStage = 0;

        stages = new Stage[]{
            new Stage(factory.getConstant(NAME, "startPos", 0), false, false, false), // just here as an init value
            new Stage(factory.getConstant(NAME, "startPos", 0), false, false, false),
            new Stage(factory.getConstant(NAME, "firstToSecondRungPos", -63), true, false, true),
            new Stage(factory.getConstant(NAME, "secondToLastRungPos", -153), false, true, false),
            new Stage(factory.getConstant(NAME, "lastPos", -180), true, false, true)
        };
    }

    public void incrementClimberStage(){ // we can't go backwards (descend rungs) using this logic, but it shouldn't really matter
        if (controlMode != ControlMode.POSITION) {
            controlMode = ControlMode.POSITION;
        }

        if(Math.abs(error) < ALLOWABLE_ERROR && currentStage < stages.length && !needsOverShoot) {
            System.out.println("incrementing climber to stage " + currentStage + " .....");
            currentStage++;
            needsOverShoot = true;
            needsClamp = true;
            outputsChanged = true;
        } else {
            System.out.println("climber not safely at stage " + currentStage + " - not incrementing stage!");
        }
    }

    public void setClimberPower(double power){
        if (controlMode != ControlMode.MANUAL) {
            controlMode = ControlMode.MANUAL;
        }
        climberPower = power;
        outputsChanged = true;
    }

    public void setTopClamp() {
        if(controlMode == ControlMode.MANUAL){
            topClamped = !topClamped;
        } else {
            System.out.println("climber not in manual mode! - not clamping");
        }
        needsClamp = true;
        outputsChanged = true;
    }

    public void setBottomClamp() {
        if(controlMode == ControlMode.MANUAL){
            bottomClamped = !bottomClamped;
        } else {
            System.out.println("climber not in manual mode! - not clamping");
        }
        needsClamp = true;
        outputsChanged = true;
    }

//    public void setClimberAngle(double angle) {
//        setClimberPosition(convertToTurretTicks(angle));
//    }

    private void positionControl(double position) {
        if(needsOverShoot) { // keep looping if we aren't past the overshoot value
            elevator.set(Position, position - 20);
            if (error < -18) {
                needsOverShoot = false;
            }
            outputsChanged = true;
        } else {
            elevator.set(Position, position);
        }
    }

    private void setClamps(boolean topClamped, boolean bottomClamped, boolean topFirst) {
        if(needsClamp){
            needsClamp = false;
            if(topFirst){
                topClamp.set(topClamped);
                Timer.delay(.5);
                bottomClamp.set(bottomClamped);
            } else {
                bottomClamp.set(bottomClamped);
                Timer.delay(.5);
                topClamp.set(topClamped);
            }
            System.out.println("setting climber clamps!");
            Timer.delay(.25);
        }
    }

    @Override
    public void readFromHardware() {
        error = elevator.getSelectedSensorPosition(0) - stages[currentStage].position;
        climberPosition = elevator.getSelectedSensorPosition(0);
//        System.out.println("climber position = " + climberPosition);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            if (controlMode == ControlMode.POSITION) {
                Stage stage = stages[currentStage];
                setClamps(stage.topClamped, stage.bottomClamped, stage.topFirst);
                positionControl(stages[currentStage].position);
            } else {
                setClamps(topClamped, bottomClamped, false);
                elevator.set(PercentOutput, climberPower);
            }
        }
    }

    public double getClimberPosition(){
        return climberPosition;
    }

    @Override
    public void zeroSensors() {
        elevator.setSelectedSensorPosition(stages[0].position, 0, Constants.kCANTimeoutMs);
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
        public final boolean topFirst;

        Stage(
            double distance,
            boolean topClamped,
            boolean bottomClamped,
            boolean topFirst
        ) {
            this.position = distance;
            this.topClamped = topClamped;
            this.bottomClamped = bottomClamped;
            this.topFirst = topFirst;
        }

        Stage() {
            this(0, false, false, false);
        }
    }
}
