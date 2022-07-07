package com.team1816.season.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.led.CANdle;
import com.team1816.lib.hardware.components.ICANdle;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import java.awt.*;
import javax.inject.Singleton;

@Singleton
public class LedManager extends Subsystem {

    public static final String NAME = "ledmanager";

    // Components
    private final ICanifier canifier;
    private final ICANdle candle;

    // State
    private boolean blinkLedOn = false;
    private boolean outputsChanged = false;
    private boolean cameraLedChanged = false;
    private int loopDelay = 0;

    private int ledR;
    private int ledG;
    private int ledB;
    private boolean cameraLedOn;

    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();
    private LedControlState controlState = LedControlState.STANDARD;
    private RobotStatus defaultStatus = RobotStatus.DISABLED;
    private float raveHue;
    private Color lastRaveColor;

    public enum LedControlState {
        RAVE,
        BLINK,
        STANDARD,
    }

    public LedManager() {
        super(NAME);
        canifier = factory.getCanifier(NAME);
        candle = factory.getCandle(NAME);
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

    public void setCameraLed(boolean cameraOn) {
        if (cameraLedOn != cameraOn) {
            cameraLedChanged = true;
            cameraLedOn = cameraOn;
        }
    }

    private void setLedColor(int r, int g, int b) {
        if (ledR != r || ledG != g || ledB != b) {
            ledR = r;
            ledG = g;
            ledB = b;
            outputsChanged = true;
        }
    }

    /**
     * @param r LED color red value (0-255)
     * @param g LED color green value (0-255)
     * @param b LED color blue value (0-255)
     */
    private void setLedColorBlink(int r, int g, int b) {
        // Period is in milliseconds
        setLedColor(r, g, b);
        controlState = LedControlState.BLINK;
        this.period = 1000;
        outputsChanged = true;
    }

    public void indicateStatus(RobotStatus status) {
        controlState = LedControlState.STANDARD;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void indicateDefaultStatus() {
        if (RAVE_ENABLED && defaultStatus != RobotStatus.DISABLED) {
            controlState = LedControlState.RAVE;
            setLedColor(0, 0, 0);
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

    private void writeToCameraLed(int r, int g, int b) {
        if (candle != null) {
            if (cameraLedOn) {
                candle.setLEDs(0, 255, 0, 0, 0, 8);
            } else {
                candle.setLEDs(r, g, b, 0, 0, 8);
            }
        }
    }

    private void writeToLed(int r, int g, int b) {
        if (candle != null) {
            candle.setLEDs(r, g, b, 0, 8, 74 - 8); // 8 == number of camera leds
        } else if (canifier != null) {
            canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
            canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
            canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            if (canifier != null || candle != null) {
                switch (controlState) {
                    case RAVE:
                        var color = Color.getHSBColor(raveHue, 1.0f, MAX / 255.0f);
                        if (!color.equals(lastRaveColor)) {
                            outputsChanged = true;
                            writeToLed(color.getRed(), color.getGreen(), color.getBlue());
                        }
                        raveHue += RAVE_SPEED;
                        break;
                    case BLINK:
                        if (System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
                            if (blinkLedOn) {
                                outputsChanged = true;
                                writeToLed(0, 0, 0);
                                blinkLedOn = false;
                            } else {
                                outputsChanged = true;
                                writeToLed(ledR, ledG, ledB);
                                blinkLedOn = true;
                            }
                            lastWriteTime = System.currentTimeMillis();
                        }
                        break;
                    case STANDARD:
                        writeToLed(ledR, ledG, ledB);
                        break;
                }
            }
        }
        if (cameraLedChanged) {
            cameraLedChanged = false;
            writeToCameraLed(ledR, ledG, ledB);
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        System.out.println("Checking LED systems");
        controlState = LedControlState.STANDARD;
        setLedColor(MAX, 0, 0); // set red
        testDelay();
        setCameraLed(true); // turn on camera
        testDelay();
        setCameraLed(false);
        testDelay();
        setLedColor(0, MAX, 0); // set green
        testDelay();
        setLedColor(0, 0, MAX); // set blue
        testDelay();
        return true;
    }

    private void testDelay() {
        loopDelay = 10;
        writeToHardware();
        Timer.delay(1.5);
    }

    @Override
    public void initSendable(SendableBuilder builder) {}

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

        final int red;
        final int green;
        final int blue;

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
