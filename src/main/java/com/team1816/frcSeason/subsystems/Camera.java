package com.team1816.frcSeason.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import java.net.*;
import java.io.*;
import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class Camera {

    private static Camera INSTANCE;

    @Inject
    private static LedManager led;

    private final String PROTOCOL_LINE = "\\|";

    // Constants
    // private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 1; // deg

    // Sockets!
    private static Socket socket;
    private static BufferedReader socketIn;
    private static PrintWriter socketOut;
    private static Boolean usingVision = true;

    public Camera() {
        try {
            socket = new Socket("localhost", 5802);
            socketOut = new PrintWriter(socket.getOutputStream(), true);
            socketIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        } catch (IOException e) {
            // (the socket crashing is urgent enough it should be sent to the driver station)
            // vision being dead is bad, but we can't afford to crash the robot by throwing
            DriverStation.reportWarning(
                "The Vision crashed with the message \"" + e.getMessage() + 
                "\", please report this immediately", 
                true
            );
            usingVision = false;
        }
    }

    private String query(String message) throws IOException {
        if (usingVision) {
            socketOut.write(message + "\n");
            socketOut.flush();
            return socketIn.readLine();
        }
        return "";
    }

    public double getDeltaXAngle() {
        if (!usingVision) return -1;
        try {
            String line = query("center_x");
            String coord = line.split(PROTOCOL_LINE)[1];
            double x = Double.parseDouble(coord);
            if (x < 0) {
                // Reset deltaX to 0 if contour not detected
                return 0;
            }
            double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
            return Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
        } catch (IOException e) {
            return -1;
        }
    }

    public double getDistance() {
        if (!usingVision) return -1;
        try {
            String line = query("distance");
            String[] parts = line.split(PROTOCOL_LINE);
            if (parts.length < 2) {
                return -1;
            }
            return Double.parseDouble(parts[1]);
        } catch (IOException e) {
            return -1;
        }
    }

    public double getRawCenterX() {
        if (!usingVision) return -1;
        try {
            String line = query("center_x");
            String coord = line.split(PROTOCOL_LINE)[1];
            return Double.parseDouble(coord);
        } catch (IOException e) {
            return -1;
        }
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        usingVision = enabled;
    }
}
