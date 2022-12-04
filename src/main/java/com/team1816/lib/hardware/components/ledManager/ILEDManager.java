package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public interface ILEDManager {
    ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count);

    ErrorCode configFactoryDefault();

    ErrorCode configStatusLedState(boolean b);

    ErrorCode configLOSBehavior(boolean b);

    ErrorCode configLEDType(CANdle.LEDStripType brg);

    ErrorCode configBrightnessScalar(double brightness);

    ErrorCode animate(Animation animation);

    ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    );
}
