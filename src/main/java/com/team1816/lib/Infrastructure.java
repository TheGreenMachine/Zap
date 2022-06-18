package com.team1816.lib;

import static com.team1816.lib.subsystems.Subsystem.factory;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
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

    private static ICompressor mCompressor;
    private static IPigeonIMU mPigeon;
    private static PowerDistribution pdh;

    private static final boolean compressorEnabled =
        factory.getConstant("compressorEnabled") > 0;
    private boolean lastCompressorOn = false;

    public Infrastructure() {
        mCompressor = factory.getCompressor();
        mPigeon = factory.getPigeon();
        mPigeon.configFactoryDefault();
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 200);
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
        pdh =
            new PowerDistribution(
                (int) factory.getConstant("pdID", 1),
                factory.getConstant("pdIsRev") > 0
                    ? PowerDistribution.ModuleType.kRev
                    : PowerDistribution.ModuleType.kCTRE
            );
    }

    public void startCompressor() { // not used because compressor currently turns on by default once robot is enabled
        if (compressorEnabled && !lastCompressorOn) {
            mCompressor.enableDigital();
            lastCompressorOn = true;
        }
    }

    public void stopCompressor() {
        if (compressorEnabled && lastCompressorOn) {
            mCompressor.disable();
            lastCompressorOn = false;
        }
    }

    public void resetPigeon(Rotation2d angle) {
        System.out.println("resetting Pigeon  - - ");
        mPigeon.setYaw(angle.getDegrees());
    }

    public IPigeonIMU getPigeon() {
        return mPigeon;
    }

    public double getYaw() {
        return mPigeon.getYaw();
    }

    public PowerDistribution getPdh() {
        return pdh;
    }
}
