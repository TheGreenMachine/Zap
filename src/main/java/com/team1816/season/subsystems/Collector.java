package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
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

    private boolean isVelocity;

    @Inject
    public Collector(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        armPiston = factory.getSolenoid(NAME, "arm");
        intakeMotor = factory.getMotor(NAME, "intake");

        isVelocity = factory.getConstant(NAME, "isVelocity", 0) > 0;

        // Constants
        double MAX_TICKS = factory.getConstant(NAME, "maxVelTicks100ms", 0);
        if (!isVelocity) {
            COLLECTING = factory.getConstant(NAME, "collecting");
            FLUSH = factory.getConstant(NAME, "flush");
            REVVING = factory.getConstant(NAME, "revving", .5);
        } else {
            COLLECTING = factory.getConstant(NAME, "collecting") * MAX_TICKS;
            FLUSH = factory.getConstant(NAME, "flush") * MAX_TICKS;
            REVVING = factory.getConstant(NAME, "revving", .5) * MAX_TICKS;
        }
    }

    public void setDesiredState(STATE state) {
        if (desiredState != state) {
            desiredState = state;
            outputsChanged = true;
        }
    }

    @Override
    public void readFromHardware() {}

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
            if (isVelocity) {
                intakeMotor.set(ControlMode.Velocity, intakeVel);
            } else {
                intakeMotor.set(ControlMode.PercentOutput, intakeVel);
            }
            armPiston.set(armDown);
        }
    }

    @Override
    public void zeroSensors() {}

    @Override
    public void stop() {}

    @Override
    public boolean testSubsystem() {
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
