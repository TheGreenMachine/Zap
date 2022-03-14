package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonSRX
    extends TalonSRX
    implements IConfigurableMotorController, IMotorSensor {

    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    private final SensorCollection mSensors;

    public LazyTalonSRX(int deviceNumber) {
        super(deviceNumber);
        mSensors = super.getSensorCollection();
    }

    @Override
    public double getLastSet() {
        return mLastSet;
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

    @Override
    public int getQuadraturePosition() {
        return mSensors.getQuadraturePosition();
    }

    @Override
    public int getPulseWidthPosition() {
        return mSensors.getPulseWidthPosition();
    }

    @Override
    public ErrorCode setQuadraturePosition(int newPosition) {
        return mSensors.setQuadraturePosition(newPosition, Constants.kLongCANTimeoutMs);
    }
}
