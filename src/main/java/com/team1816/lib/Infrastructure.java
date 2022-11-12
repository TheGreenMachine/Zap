package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.Constants;
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

    private double xAccel, yAccel, zAccel;
    private double loopCount;

    private static boolean compressorEnabled;
    private static boolean compressorIsOn = false;

    @Inject
    public Infrastructure(RobotFactory factory) {
        compressor = factory.getCompressor();
        pigeon = factory.getPigeon();
        pd = factory.getPd();
        compressorEnabled = factory.isCompressorEnabled();

        xAccel = yAccel = zAccel = loopCount = 0;
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

    public void update() {
        if (loopCount > 5) {
            loopCount = 0;
            double[] accel = pigeon.getAcceleration();
            xAccel = accel[0] * Constants.gravitationalAccelerationConstant / 16384;
            yAccel = accel[1] * Constants.gravitationalAccelerationConstant / 16384;
            zAccel = -accel[2] * Constants.gravitationalAccelerationConstant / 16384;
        } else {
            loopCount++;
        }
    }


    public Double[] getAcceleration() {
        return new Double[] {xAccel, yAccel, zAccel};
    }

    public double getXAcceleration() {
        return getAcceleration()[0];
    }

    public double getYAcceleration() {
        return getAcceleration()[1];
    }

    public double getZAcceleration() {
        return getAcceleration()[2];
    }

    public PowerDistribution getPd() {
        return pd;
    }

    public void simulateGyro(double radianOffsetPerLoop, double gyroDrift) {
        pigeon.setYaw(getYaw() + radianOffsetPerLoop + gyroDrift);
    }
}
