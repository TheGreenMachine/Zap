package com.team1816.lib.hardware.components;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleStatusFrame;

public interface ICANdle {
    ErrorCode setStatusFramePeriod(
        CANdleStatusFrame caNdleStatusFrame_status_4_controlTelem,
        int canMaxStatus,
        int i
    );

    ErrorCode configFactoryDefault();

    ErrorCode configStatusLedState(boolean b);

    ErrorCode configLOSBehavior(boolean b);

    ErrorCode configLEDType(CANdle.LEDStripType brg);

    ErrorCode configBrightnessScalar(double brightness);

    ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count);

    ErrorCode animate(Animation animation);
}
