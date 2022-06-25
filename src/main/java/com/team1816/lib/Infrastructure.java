package com.team1816.lib;

import static com.team1816.lib.subsystems.Subsystem.factory;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Subsystem housing compressor and pigeon - should we add pcm/pdh here?
 */

@Singleton
public class Infrastructure {

    // Components
    private static ICompressor compressor = factory.getCompressor();
    private static IPigeonIMU pigeon = factory.getPigeon();
    private static PowerDistribution pd = factory.getPd();

    private static final boolean compressorEnabled = factory.isCompressorEnabled();
    private static boolean compressorIsOn = false;

    public Infrastructure() {}

    public static void startCompressor() { // not used because compressor currently turns on by default once robot is enabled
        if (compressorEnabled && !compressorIsOn) {
            compressor.enableDigital();
            compressorIsOn = true;
        }
    }

    public static void stopCompressor() {
        if (compressorEnabled && compressorIsOn) {
            compressor.disable();
            compressorIsOn = false;
        }
    }

    public static void resetPigeon(Rotation2d angle) {
        System.out.println("resetting Pigeon");
        pigeon.setYaw(angle.getDegrees());
    }

    public static IPigeonIMU getPigeon() {
        return pigeon;
    }

    public static double getYaw() {
        return pigeon.getYaw();
    }

    public static PowerDistribution getPd() {
        return pd;
    }

    public static void simulateGyro(double radianOffsetPerLoop, double gyroDrift) {
        pigeon.setYaw(getYaw() + radianOffsetPerLoop + gyroDrift);
    }
}
