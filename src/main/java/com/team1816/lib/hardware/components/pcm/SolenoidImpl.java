package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidImpl extends Solenoid implements ISolenoid {

    public SolenoidImpl(final PneumaticsModuleType moduleType, final int channel) {
        super(moduleType, channel);
    }

    public SolenoidImpl(
        final int module,
        final PneumaticsModuleType moduleType,
        final int channel
    ) {
        super(module, moduleType, channel);
    }
}
