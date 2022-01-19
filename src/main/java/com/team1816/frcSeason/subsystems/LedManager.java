package com.team1816.frcSeason.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team1816.lib.hardware.components.ICanifier;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Singleton;
import java.awt.*;

@Singleton
public class LedManager extends Subsystem {

    public static final String NAME = "ledmanager";

    private static LedManager INSTANCE;

    public enum LedControlState {
        RAVE,
        BLINK,
        STANDARD,
    }

    // Components
    private final ICanifier canifier;
    private final ICanifier cameraCanifier;
    private final DigitalOutput cameraLed;

    // State
    private LedControlState controlState = LedControlState.STANDARD;
    private RobotStatus defaultStatus = RobotStatus.DISABLED;
    private boolean outputsChanged = true;

    private int ledR;
    private int ledG;
    private int ledB;
    private boolean cameraLedOn;

    private boolean blinkLedOn = false;
    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();

    private float raveHue = 0f;

    // Constants
    private static final boolean RAVE_ENABLED =
        factory.getConstant(NAME, "raveEnabled") > 0;
    private static final double RAVE_SPEED = factory.getConstant(NAME, "raveSpeed", 0.01);
    private static final int MAX = (int) factory.getConstant(NAME, "maxLevel", 255);

    public LedManager() {
        super(NAME);
        this.canifier = factory.getCanifier(NAME);
        this.cameraCanifier = factory.getCanifier("camera");
        this.cameraLed = new DigitalOutput((int) factory.getConstant(NAME, "cameraLed", 1));

        configureCanifier(canifier);
        configureCanifier(cameraCanifier);

        this.ledR = 0;
        this.ledG = 0;
        this.ledB = 0;

        this.cameraLedOn = false;
    }

    private void configureCanifier(ICanifier canifier) {
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 255, 10);
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 255, 10);
    }

    public void setCameraLed(boolean cameraLedOn) {
        if (this.cameraLedOn != cameraLedOn) {
            this.cameraLedOn = cameraLedOn;
            outputsChanged = true;
        }
    }

    public void setLedColor(int r, int g, int b) {
        if (this.ledR != r || this.ledG != g || this.ledB != b) {
            this.ledR = r;
            this.ledG = g;
            this.ledB = b;
            outputsChanged = true;
        }
    }

    public void setControlState(LedControlState controlState) {
        this.controlState = controlState;
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
        setControlState(LedControlState.BLINK);
        this.period = period;
        outputsChanged = true;
    }

    private void setLedColorBlink(int r, int g, int b) {
        // Default period of 1 second
        setLedColorBlink(r, g, b, 1000);
    }

    public void indicateStatus(RobotStatus status) {
        setControlState(LedControlState.STANDARD);
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
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

    public RobotStatus getDefaultStatus() {
        return defaultStatus;
    }

    public int[] getLedColor() {
        return new int[] { ledR, ledG, ledB };
    }

    public LedControlState getControlState() {
        return controlState;
    }

    public double getPeriod() {
        return period;
    }

    private void writeLedHardware(int r, int g, int b) {
        canifier.setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
    }

    @Override
    public void writePeriodicOutputs() {
        if (cameraLed != null) {
            if (outputsChanged) {
                cameraLed.set(cameraLedOn);
//                cameraCanifier.setLEDOutput(
//                    cameraLedOn ? 1 : 0,
//                    CANifier.LEDChannel.LEDChannelB
//                );
            }
        }
        if (canifier != null) {
            switch (controlState) {
                case RAVE:
                    var color = Color.getHSBColor(raveHue, 1.0f, 1.0f);
                    writeLedHardware(color.getRed(), color.getGreen(), color.getBlue());
                    raveHue += RAVE_SPEED;
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
                    if (outputsChanged) {
                        writeLedHardware(ledR, ledG, ledB);
                        outputsChanged = false;
                    }
                    break;
            }
        }
    }

    @Override
    public void stop() {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        super.registerEnabledLoops(mEnabledLooper);
        mEnabledLooper.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {}

                @Override
                public void onLoop(double timestamp) {
                    LedManager.this.writePeriodicOutputs();
                }

                @Override
                public void onStop(double timestamp) {}
            }
        );
    }

    @Override
    public boolean checkSystem() {
        System.out.println("Warning: checking LED systems");
        Timer.delay(3);

        writeLedHardware(255, 0, 0);
        Timer.delay(0.4);
        writeLedHardware(0, 255, 0);
        Timer.delay(0.4);
        writeLedHardware(0, 0, 255);
        Timer.delay(0.4);
        writeLedHardware(0, 0, 0);

        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}

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
