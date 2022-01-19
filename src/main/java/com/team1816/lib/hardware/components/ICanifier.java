package com.team1816.lib.hardware.components;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;

public interface ICanifier {
    void setLEDOutput(double percentOutput, CANifier.LEDChannel ledChannel);

    ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    );

    ErrorCode setStatusFramePeriod(CANifierStatusFrame statusFrame, int periodMs);
}
