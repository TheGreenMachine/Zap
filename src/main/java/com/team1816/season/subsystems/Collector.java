package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Collector extends Subsystem {

    private static final String NAME = "collector";

    // Components
    private final ISolenoid armPiston;
    private final IGreenMotor intakeMotor;

    // State
    private double intakeVel;
    private boolean armDown;
    private boolean outputsChanged = false;
    private STATE desiredState = STATE.STOP;

    private final double COLLECTING;
    private final double FLUSH;
    private final double REVVING;

    public Collector() {
        super(NAME);
        armPiston = factory.getSolenoid(NAME, "arm");
        intakeMotor = factory.getMotor(NAME, "intake");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        intakeMotor.config_kP(0, config.kP, 100);
        intakeMotor.config_kI(0, config.kI, 100);
        intakeMotor.config_kD(0, config.kD, 100);
        intakeMotor.config_kF(0, config.kF, 100);

        COLLECTING = factory.getConstant(NAME, "collecting");
        FLUSH = factory.getConstant(NAME, "flush");
        REVVING = factory.getConstant(NAME, "revving", .5);
    }

    public void setDesiredState(STATE state) {
        if (desiredState != state) {
            desiredState = state;
            outputsChanged = true;
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP:
                    intakeVel = 0;
                    armDown = false;
                    break;
                case COLLECTING:
                    intakeVel = COLLECTING;
                    armDown = true;
                    break;
                case REVVING:
                    intakeVel = REVVING;
                    armDown = false;
                    break;
                case FLUSH:
                    intakeVel = FLUSH;
                    armDown = true;
                    break;
            }
            intakeMotor.set(ControlMode.Velocity, intakeVel);
            armPiston.set(armDown);
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        setDesiredState(STATE.COLLECTING);
        Timer.delay(1);
        if (
            armDown != armPiston.get() &&
            Math.abs(intakeMotor.getSelectedSensorVelocity(0) - intakeVel) > 1000
        ) {
            return false;
        }
        setDesiredState(STATE.STOP);

        return true;
    }

    public enum STATE {
        STOP,
        COLLECTING,
        REVVING,
        FLUSH,
    }
}
