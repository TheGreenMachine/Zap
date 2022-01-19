package com.team1816.lib.hardware.components;

import com.ctre.phoenix.CANifier;

public class CanifierImpl extends CANifier implements ICanifier {

    /**
     * Constructor.
     *
     * @param deviceId The CAN Device ID of the CANifier.
     */
    public CanifierImpl(int deviceId) {
        super(deviceId);
    }
}
