package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyTalonFX extends TalonFX implements IConfigurableMotorController {

    protected double mLastSet = Double.NaN;
    protected String mName = "";
    protected ControlMode mLastControlMode = null;

    public LazyTalonFX(int deviceNumber, String name, String canBus) {
        super(deviceNumber, canBus);
        mName = name;
    }

    @Override
    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public String getName() {
        return mName;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            if (!super.hasResetOccurred()) {
                mLastSet = value;
                mLastControlMode = mode;
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
