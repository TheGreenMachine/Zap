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

    protected double lastSet = Double.NaN;
    protected String name = "";
    protected ControlMode lastControlMode = null;
    private final SensorCollection sensors;

    public LazyTalonSRX(int deviceNumber, String motorName) {
        super(deviceNumber);
        sensors = super.getSensorCollection();
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

    @Override
    public int getQuadraturePosition() {
        return sensors.getQuadraturePosition();
    }

    @Override
    public int getPulseWidthPosition() {
        return sensors.getPulseWidthPosition();
    }

    @Override
    public ErrorCode setQuadraturePosition(int newPosition) {
        return sensors.setQuadraturePosition(newPosition, Constants.kLongCANTimeoutMs);
    }
}
