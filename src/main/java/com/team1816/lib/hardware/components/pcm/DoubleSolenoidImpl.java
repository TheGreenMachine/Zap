package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class DoubleSolenoidImpl extends DoubleSolenoid implements IDoubleSolenoid {

    public DoubleSolenoidImpl(
        final PneumaticsModuleType moduleType,
        final int forwardChannel,
        final int reverseChannel
    ) {
        super(moduleType, forwardChannel, reverseChannel);
    }

    public DoubleSolenoidImpl(
        final int module,
        final PneumaticsModuleType moduleType,
        final int forwardChannel,
        final int reverseChannel
    ) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }
}
