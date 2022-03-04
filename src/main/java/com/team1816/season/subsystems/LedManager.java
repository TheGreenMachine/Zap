package com.team1816.season.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Singleton;

@Singleton
public class LedManager extends Subsystem {

    public static final String NAME = "ledmanager";

    private static LedManager INSTANCE;

    // Components
    private final ICanifier canifier;
    private final CANdle candle;

    // State
    private boolean blinkLedOn = false;
    private boolean outputsChanged = true;
    private boolean cameraLedChanged = false;

    private int ledR;
    private int ledG;
    private int ledB;
    private boolean cameraLedOn;

    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();
    private LedControlState controlState = LedControlState.STANDARD;
    private RobotStatus defaultStatus = RobotStatus.DISABLED;

    public enum LedControlState {
        RAVE,
        BLINK,
        STANDARD,
    }

    public LedManager() {
        super(NAME);
        canifier = factory.getCanifier(NAME);
        candle =  factory.getCandle(NAME,22);

        configureCanifier(canifier);
        configureCandle();

        ledR = 0;
        ledG = 0;
        ledB = 0;

        cameraLedOn = false;
    }

    private void configureCandle() {
        if (candle == null) return;
        candle.configStatusLedState(true);
        candle.configLOSBehavior(true);
        candle.configLEDType(CANdle.LEDStripType.BRG);
        candle.configBrightnessScalar(1);
    }

    private void configureCanifier(ICanifier canifier) {
        if (canifier == null) return;
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 255, 10);
    }

    public void setCameraLed(boolean cameraLedOn) {
        if (this.cameraLedOn != cameraLedOn) {
            this.cameraLedOn = cameraLedOn;
            cameraLedChanged = true;
        }
    }

    public void setLedColor(int r, int g, int b) {
        if (ledR != r || ledG != g || ledB != b) {
            ledR = r;
            ledG = g;
            ledB = b;
            outputsChanged = true;
        }
    }

    /**
     * @param r      LED color red value (0-255)
     * @param g      LED color green value (0-255)
     * @param b      LED color blue value (0-255)
     * @param period milliseconds
     */
    private void setLedColorBlink(int r, int g, int b, int period) {
        // Period is in milliseconds
        setLedColor(r, g, b);
        controlState = LedControlState.BLINK;
        this.period = period;
        outputsChanged = true;
    }

    private void setLedColorBlink(int r, int g, int b) {
        // Default period of 1 second
        setLedColorBlink(r, g, b, 1000);
    }

    public void indicateStatus(RobotStatus status) {
        controlState = LedControlState.STANDARD;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void setControlState(LedControlState controlState) {
        this.controlState = controlState;
    }

    public void indicateDefaultStatus() {
        if (RAVE_ENABLED && defaultStatus != RobotStatus.DISABLED) {
            setControlState(LedControlState.RAVE);
        } else {
            indicateStatus(defaultStatus);
        }
    }

    public void blinkStatus(RobotStatus status) {
        setLedColorBlink(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void setDefaultStatus(RobotStatus defaultStatus) {
        this.defaultStatus = defaultStatus;
        indicateDefaultStatus();
    }

    public double getPeriod() {
        return period;
    }

    private void writeLedHardware(int r, int g, int b) {
        if (candle != null) {
            candle.setLEDs(r, g, b, 0, 8, 66);
            // back to back writes cancel the first output, so we need to give candle time to write
            if (cameraLedChanged) {
                Timer.delay(.1);
                candle.setLEDs(0, cameraLedOn ? 255 : 0, 0, 0, 0, 8);
            }
        }
        if (canifier != null && outputsChanged) {
            canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
            canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
            canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged || cameraLedChanged) {
            if (canifier != null || candle != null) {
                switch (controlState) {
                    case RAVE:
                        candle.animate(new RainbowAnimation(MAX / 255.0, RAVE_SPEED, 74));
                        break;
                    case BLINK:
                        if (System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
                            if (blinkLedOn) {
                                writeLedHardware(0, 0, 0);
                                blinkLedOn = false;
                            } else {
                                writeLedHardware(ledR, ledG, ledB);
                                blinkLedOn = true;
                            }
                            lastWriteTime = System.currentTimeMillis();
                        }
                        break;
                    case STANDARD:
                        writeLedHardware(ledR, ledG, ledB);
                        break;
                }
            }
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        super.registerEnabledLoops(mEnabledLooper);
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                }

                @Override
                public void onLoop(double timestamp) {
                    LedManager.this.writeToHardware();
                }

                @Override
                public void onStop(double timestamp) {
                }
            }
        );
    }

    @Override
    public boolean checkSystem() {
        System.out.println("Warning: checking LED systems");
        writeLedHardware(MAX, 0, 0);
        Timer.delay(2);
        setCameraLed(true);
        writeLedHardware(0, MAX, 0);
        Timer.delay(2);
        setCameraLed(false);
        writeLedHardware(0, 0, MAX);
        Timer.delay(2);
        writeLedHardware(0, 0, 0);
        Timer.delay(2);
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    private static final boolean RAVE_ENABLED =
        factory.getConstant(NAME, "raveEnabled") > 0;
    private static final double RAVE_SPEED = factory.getConstant(NAME, "raveSpeed", 1.0);
    private static final int MAX = (int) factory.getConstant(NAME, "maxLevel", 255);

    public enum RobotStatus {
        ENABLED(0, MAX, 0), // green
        DISABLED(MAX, MAX / 5, 0), // orange
        ERROR(MAX, 0, 0), // red
        AUTONOMOUS(0, MAX, MAX), // cyan
        ENDGAME(0, 0, MAX), // blue
        SEEN_TARGET(MAX, 0, MAX), // magenta
        ON_TARGET(MAX, 0, 20), // deep magenta
        DRIVETRAIN_FLIPPED(MAX, MAX, 0), // yellow,
        MANUAL_TURRET(MAX, MAX, MAX), // white
        OFF(0, 0, 0); // off

        int red;
        int green;
        int blue;

        RobotStatus(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }
    }
}
