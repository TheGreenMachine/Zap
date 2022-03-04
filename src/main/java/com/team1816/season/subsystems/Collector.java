package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

@Singleton
public class Collector extends Subsystem {

    // the main reason we're messing with spark motors in the factory is to use a neo motor on our collector.
    // this means we'll be making a spark controller here (which probably means calling IMotorControllerEnhanced?)

    private static final String NAME = "collector";

    // Components
    private final ISolenoid armPiston;
    private final IMotorControllerEnhanced intake;
    //private final CANSparkMax intake;

    // State
    private double intakePow;
    private boolean armDown;
    private boolean outputsChanged = false;
    private COLLECTOR_STATE state = COLLECTOR_STATE.STOP;

    private double REVVING = factory.getConstant(NAME, "revving");
    private double COLLECTING = factory.getConstant(NAME, "collecting");
    private double FLUSH = factory.getConstant(NAME, "flush");

    private double actualVelocity;

    public Collector() {
        super(NAME);
        armPiston = factory.getSolenoid(NAME, "arm");
        intake = factory.getMotor(NAME, "intake");
        //        intake.configSupplyCurrentLimit(
        //            new SupplyCurrentLimitConfiguration(true, 25, 0, 0),
        //            Constants.kCANTimeoutMs
        //        );
    }

    public boolean isArmDown() {
        return this.armDown;
    }

    public double getIntakePow() {
        return this.intakePow;
    }

    public double getActualVelocity() {
        return this.actualVelocity;
    }

    private void setArm(boolean down) {
        this.armDown = down;
        this.outputsChanged = true;
    }

    public void setState(COLLECTOR_STATE state) {
        this.state = state;
        System.out.println("COLLECTOR STATE IS CHANGED TO " + state);
        outputsChanged = true;
    }

    @Override
    public void readFromHardware() {
        //        this.actualVelocity = intake.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            switch (state) {
                case STOP:
                    intakePow = 0;
                    armDown = false;
                    break;
                case REVVING:
                    intakePow = REVVING;
                    armDown = false;
                    break;
                case COLLECTING:
                    intakePow = COLLECTING;
                    armDown = true;
                    break;
                case FLUSH:
                    intakePow = FLUSH;
                    armDown = true; // NOT SURE IF WE WANT COLLECTOR DOWN HERE
                    break;
            }
            intake.set(ControlMode.PercentOutput, intakePow);
            this.armPiston.set(armDown);

            this.outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    public enum COLLECTOR_STATE {
        STOP,
        REVVING,
        COLLECTING,
        FLUSH,
    }
}
