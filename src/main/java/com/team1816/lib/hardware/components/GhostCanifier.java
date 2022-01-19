package com.team1816.lib.hardware.components;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;

public class GhostCanifier implements ICanifier {

    @Override
    public void setLEDOutput(double percentOutput, CANifier.LEDChannel ledChannel) {
        // no-op
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(CANifierStatusFrame statusFrame, int periodMs) {
        return ErrorCode.OK;
    }
}
