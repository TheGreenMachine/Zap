package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class CanifierImpl extends CANifier implements ILEDManager {

    /**
     * Constructor.
     *
     * @param deviceId The CAN Device ID of the CANifier.
     */
    public CanifierImpl(int deviceId) {
        super(deviceId);
    }

    @Override
    public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatusLedState(boolean b) {
        return null;
    }

    @Override
    public ErrorCode configLOSBehavior(boolean b) {
        return null;
    }

    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType brg) {
        return null;
    }

    @Override
    public ErrorCode configBrightnessScalar(double brightness) {
        return null;
    }

    @Override
    public ErrorCode animate(Animation animation) {
        return null;
    }
}
