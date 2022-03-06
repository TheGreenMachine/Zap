package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;

@Singleton
public class Collector extends Subsystem {

    private static final String NAME = "collector";

    // Components
    private final ISolenoid armPiston;
    private final IMotorControllerEnhanced intake;

    // State
    private double intakePow;
    private boolean armDown;
    private boolean outputsChanged = false;
    private COLLECTOR_STATE state = COLLECTOR_STATE.STOP;

    private final double REVVING;
    private final double COLLECTING;
    private final double FLUSH;

    private double actualVelocity;

    public Collector() {
        super(NAME);
        armPiston = factory.getSolenoid(NAME, "arm");
        intake = factory.getMotor(NAME, "intake");

        REVVING = factory.getConstant(NAME, "revving");
        COLLECTING = factory.getConstant(NAME, "collecting");
        FLUSH = factory.getConstant(NAME, "flush");
    }

    public void setState(COLLECTOR_STATE state) {
        if(this.state != state){
            this.state = state;
            System.out.println("COLLECTOR STATE IS CHANGED TO " + state);
            outputsChanged = true;
        }
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
                    armDown = true; // do we want arm down for this? pending for build team opinion...
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
