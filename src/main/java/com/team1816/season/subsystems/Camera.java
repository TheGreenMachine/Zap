package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import java.io.*;
import java.net.*;

@Singleton
public class Camera extends Subsystem {

    private static final String NAME = "camera";
    private static Camera INSTANCE;

    private final String PROTOCOL_LINE = "\\|";

    // Components
    @Inject
    static LedManager led;

    // Constants
    // private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 1; // deg

    // Sockets!
    private Socket socket;
    private BufferedReader socketIn;
    private PrintWriter socketOut;
    private Boolean usingVision = true;
    private long needsReconnect = 0;

    public Camera() {
        super(NAME);
        socketConnect();
    }

    private String query(String message) throws IOException {
        if (needsReconnect != 0) return "";
        if (usingVision) {
            socketOut.write(message);
            socketOut.flush();
            return socketIn.readLine();   
        }
        return "";
    }

    private boolean socketConnect() {
        try {
            socket = new Socket("10.18.16.16", 5802);
            //socket.connect(new InetSocketAddress(InetAddress.getLocalHost(), 5802), 10);
            socketIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            socketOut = new PrintWriter(socket.getOutputStream(), true);
        } catch (IOException e) {
            System.out.println("socket Connect error: " + e.getMessage());
            needsReconnect = System.currentTimeMillis();
            return false;
        }
        return true;
    }

    public double getDeltaXAngle() {
        if (!usingVision) return 0;
        try {
            String line = query("center_x");
            String[] coords = line.split(PROTOCOL_LINE);
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
        } catch (IOException e) {
            System.out.println(e.getMessage());
            needsReconnect = System.currentTimeMillis();
            return 0;
        }
    }

    public double getDistance() {
        if (!usingVision) return 0;
        try {
            String line = query("distance");
            System.out.println(line);
            String[] parts = line.split(PROTOCOL_LINE);
            if (parts.length < 2) {
                System.out.println(parts);
                return 0;
            }
            System.out.println("CAMERA LINE 91: getDistance() " + Double.parseDouble(parts[1]));
            return Double.parseDouble(parts[1]);
        } catch (IOException e) {
            System.out.println("CAMERA EXCEPTION GET DISTANCE LINE 94 " + e.getMessage());
            needsReconnect = System.currentTimeMillis();
            return 0;
        }
    }

    public double getRawCenterX() {
        if (!usingVision) return 0;
        try {
            String line = query("center_x");
            String[] coords = line.split(PROTOCOL_LINE);
            if (coords.length < 2) {
                return 0;
            }
            return Double.parseDouble(coords[1]);
        } catch (IOException e) {
            needsReconnect = System.currentTimeMillis();
            return 0;
        }
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        usingVision = enabled;
    }

    public boolean checkSystem() {
        return needsReconnect != 0;
    }

    public void stop() {
        try {
            socket.close();
        } catch (IOException e) {}
    }

    public void readFromHardware() {
        // if more than 200ms, reconnect
        if (needsReconnect != 0 && (System.currentTimeMillis() - needsReconnect) >= 200) {
            //            System.out.println("Reconnect attempt at " + System.currentTimeMillis());
            if (socketConnect()) {
                needsReconnect = 0;
            }
        }
    }
}
