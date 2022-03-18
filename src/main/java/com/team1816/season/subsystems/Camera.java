package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.vision.VisionSocket;

@Singleton
public class Camera extends Subsystem {

    private static final String NAME = "camera";

    public final VisionSocket socket = new VisionSocket();

    // Components
    @Inject
    static LedManager led;

    // Constants
    // private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 0.2; // deg

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    public double getDeltaXAngle() {
        String[] coords = socket.request("center_x");
        if (coords.length < 2) {
            return 0;
        }
        double x = Double.parseDouble(coords[1]);
        if (x < 0) {
            // Reset deltaX to 0 if contour not detected
            return 0;
        }
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        return Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
    }

    public double getDistance() {
        String[] parts = socket.request("distance");
        if (parts.length < 2) {
            for (int i = 0; i< parts.length; i++) {
                System.out.print(parts[i]+" ");
            }
            System.out.println();
            return 0;
        }
        System.out.println("CAMERA: getDistance() " + Double.parseDouble(parts[1]));
        return Double.parseDouble(parts[1]);
    }

    public double getRawCenterX() {
        String[] coords = socket.request("center_x");
        if (coords.length < 2) {
            return 0;
        }
        return Double.parseDouble(coords[1]);
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        socket.setEnabled(enabled);
    }

    public boolean checkSystem() {
        return true;
    }

    public void stop() {
        socket.close();
    }

    public void readFromHardware() {
        if (socket.shouldReconnect()) {
            socket.connect();
        }
    }
}
