package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;

public class CANdleImpl extends CANdle implements ILEDManager {

    public CANdleImpl(Integer candle, String canivoreBusName) {
        super(candle, canivoreBusName);
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
