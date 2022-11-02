package com.team1816.lib.hardware.components.led;

import com.ctre.phoenix.led.CANdle;

public class CANdleImpl extends CANdle implements ICANdle {

    public CANdleImpl(Integer candle, String canivoreBusName) {
        super(candle, canivoreBusName);
    }
}
