package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Singleton;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
    private final CANSparkMax intake;

    // State
    private double intakePow;
    private boolean armDown;
    private boolean outputsChanged = false;

    private boolean isRaising;
    private double startTime;

    private double actualVelocity;

    public Collector() {
        super(NAME);
        this.armPiston = factory.getSolenoid(NAME, "arm");
        this.intake =  new CANSparkMax(23, CANSparkMaxLowLevel.MotorType.kBrushless); // factory.getMotor(NAME, "intake");

        intake.setSmartCurrentLimit( 25
//            new SupplyCurrentLimitConfiguration(true, 25, 0, 0),
//            Constants.kCANTimeoutMs
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

    public void setArm(boolean down) {
        this.armDown = down;
        this.outputsChanged = true;
    }

    public void setIntakePow(double intakePower) {
        this.intakePow = intakePower;
        outputsChanged = true;
    }

    public void setDeployed(boolean down, boolean reverse) {
        isRaising = !down;
        if (down) {
            if (reverse) {
                setIntakePow(0.60);
            } else {
                setIntakePow(-0.60);
            }
            setArm(true);
        } else {
            startTime = Timer.getFPGATimestamp();
            setArm(false);
        }
    }

    @Override
    public void readPeriodicInputs() {
//        this.actualVelocity = intake.getSelectedSensorVelocity(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (isRaising) {
            if ((Timer.getFPGATimestamp() - startTime) > 1) {
                System.out.println(
                    "Raising timer passed at : " + (Timer.getFPGATimestamp() - startTime)
                );
                setIntakePow(0);
                isRaising = false;
            }
        }
        if (outputsChanged) {
            this.armPiston.set(armDown);
            System.out.println("arm is going up ");
//            this.intake.set(ControlMode.PercentOutput, intakePow);
            intake.set(intakePow);
            this.outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
