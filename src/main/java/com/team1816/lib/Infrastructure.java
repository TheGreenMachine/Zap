package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Subsystem housing compressor and pigeon - should we add pcm/pdh here?
 */

@Singleton
public class Infrastructure {

    // Components
    private static ICompressor compressor;
    private static IPigeonIMU pigeon;
    private static PowerDistribution pd;

    private static boolean compressorEnabled;
    private static boolean compressorIsOn = false;

    @Inject
    public Infrastructure(RobotFactory factory) {
        compressor = factory.getCompressor();
        pigeon = factory.getPigeon();
        pd = factory.getPd();
        compressorEnabled = factory.isCompressorEnabled();
    }

    public void startCompressor() { // not used because compressor currently turns on by default once robot is enabled
        if (compressorEnabled && !compressorIsOn) {
            compressor.enableDigital();
            compressorIsOn = true;
        }
    }

    public void stopCompressor() {
        if (compressorEnabled && compressorIsOn) {
            compressor.disable();
            compressorIsOn = false;
        }
    }

    public void resetPigeon(Rotation2d angle) {
        System.out.println("resetting Pigeon");
        pigeon.setYaw(angle.getDegrees());
    }

    public IPigeonIMU getPigeon() {
        return pigeon;
    }

    public double getYaw() {
        return pigeon.getYaw();
    }

    public PowerDistribution getPd() {
        return pd;
    }

    public void simulateGyro(double radianOffsetPerLoop, double gyroDrift) {
        pigeon.setYaw(getYaw() + radianOffsetPerLoop + gyroDrift);
    }
}
