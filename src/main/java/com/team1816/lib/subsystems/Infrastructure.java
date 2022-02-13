package com.team1816.lib.subsystems;

import com.google.inject.Inject;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.season.subsystems.Superstructure;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Subsystem to ensure the compressor never runs while the superstructure moves
 */
public class Infrastructure extends Subsystem {

    private static Infrastructure mInstance;

    @Inject
    private Superstructure mSuperstructure;

    private ICompressor mCompressor;

    private boolean mIsManualControl = false;
    private static final boolean COMPRESSOR_ENABLED =
        factory.getConstant("compressorEnabled") > 0;
    private boolean lastCompressorOn = true;

    public Infrastructure() {
        super("Infrastructure");
        mCompressor = factory.getCompressor();
        if (factory.getConstant("compressorEnabled") > 0) {
            mCompressor.stop();
        }
    }

    @Override
    public boolean isImplemented() {
        return true;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Infrastructure.this) {
                        boolean superstructureMoving = !mSuperstructure.isAtDesiredState();
                        if (!(factory.getConstant("compressorEnabled") > 0)) {
                            if (superstructureMoving || !mIsManualControl) {
                                if (lastCompressorOn) {
                                    stopCompressor();
                                    lastCompressorOn = false;
                                }
                            } else {
                                if (!lastCompressorOn) {
                                    startCompressor();
                                    lastCompressorOn = true;
                                }
                            }
                        } else {
                            stopCompressor();
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {}
            }
        );
    }

    public synchronized void setIsManualControl(boolean isManualControl) {
        mIsManualControl = isManualControl;

        if (mIsManualControl) {
            startCompressor();
        }
    }

    public synchronized boolean isManualControl() {
        return mIsManualControl;
    }

    private void startCompressor() {
        if (COMPRESSOR_ENABLED) {
            mCompressor.start();
        }
    }

    private void stopCompressor() {
        if (COMPRESSOR_ENABLED) {
            mCompressor.stop();
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
