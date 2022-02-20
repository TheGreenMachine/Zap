package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Collector extends Subsystem {
    // the main reason we're messing with spark motors in the factory is to use a neo motor on our collector.
    // this means we'll be making a spark controller here (which probably means no calling IMotorControllerEnhanced?)

    private static final String NAME = "collector";

    // Components
    private final ISolenoid armPiston;
    private final IMotorControllerEnhanced intake;

    // State
    private double intakePow;
    private boolean armDown;
    private boolean outputsChanged = false;
    private COLLECTOR_STATE state = COLLECTOR_STATE.STOP;

    private boolean isRaising;
    private double startTime;

    private double actualVelocity;

    public Collector() {
        super(NAME);
        this.armPiston = factory.getSolenoid(NAME, "arm");
        this.intake = factory.getMotor(NAME, "intake"); // factory.getMotor(NAME, "intake");

        intake.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 25, 0, 0),
            Constants.kCANTimeoutMs
        );
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

    public void setState(COLLECTOR_STATE state){
        this.state = state;
        outputsChanged = true;
    }

    @Override
    public void readFromHardware() {
        this.actualVelocity = intake.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
//        if (isRaising) {
//            if ((Timer.getFPGATimestamp() - startTime) > 1) {
//                System.out.println(
//                    "Raising timer passed at : " + (Timer.getFPGATimestamp() - startTime)
//                );
//                setIntakePow(0);
//                isRaising = false;
//            }
//        }
        if (outputsChanged) {
            switch (state){
                case STOP:
                    System.out.println("arm is going up ");
                    intakePow = 0;
                    armDown = false;
                case REVVING:
                    intakePow = 0.25;
                    armDown = false;
                case COLLECTING:
                    intakePow = 1;
                    armDown = true;
                case FLUSH:
                    intakePow = -1;
                    armDown = true; // NOT SURE IF WE WANT COLLECTOR DOWN HERE
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

    public enum COLLECTOR_STATE{
        STOP,
        REVVING,
        COLLECTING,
        FLUSH
    }
}
