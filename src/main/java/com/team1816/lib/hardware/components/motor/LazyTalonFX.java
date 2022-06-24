package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyTalonFX extends TalonFX implements IConfigurableMotorController {

    protected double lastSet = Double.NaN;
    protected String name = "";
    protected ControlMode lastControlMode = null;

    public LazyTalonFX(int deviceNumber, String motorName, String canBus) {
        super(deviceNumber, canBus);
        name = motorName;
    }

    @Override
    public double getLastSet() {
        return lastSet;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != lastSet || mode != lastControlMode) {
            if (!super.hasResetOccurred()) {
                lastSet = value;
                lastControlMode = mode;
                super.set(mode, value);
            } else {
                DriverStation.reportError("MOTOR " + getDeviceID() + " HAS RESET", false);
            }
        }
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        return super.configAllSettings(allConfigs, timeoutMs);
    }
}
