package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class GhostCANdle implements ILEDManager {

    @Override
    public ErrorCode configFactoryDefault() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configStatusLedState(boolean b) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLOSBehavior(boolean b) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType brg) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configBrightnessScalar(double brightness) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode animate(Animation animation) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    ) {
        return null;
    }
}
