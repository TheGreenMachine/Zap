package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import com.team1816.lib.vision.VisionSocket;
import java.util.ArrayList;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class Camera extends Subsystem {

    private static final String NAME = "camera";

    public final VisionSocket socket = new VisionSocket();

    public static boolean cameraEnabled;

    public double shuffleBoardDistance = -1;

    // Components
    @Inject
    static RobotState state;
    @Inject
    static LedManager led;

    // Constants
    // private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 0.2; // deg
    private int loops = 0;
    private ArrayList<Double> distances = new ArrayList<Double>();

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    private double parseDeltaX(double x) {
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        double base = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
        return base;
    }

    public double getDeltaXAngle() {
        return state.visionPoint.deltaX;
    }

    public double getDistance() {
        if(factory.getConstant(NAME, "useShuffleboard", 0) > 0){
            return shuffleBoardDistance;
        }
        double deviation = factory.getConstant(NAME, "deviation", 0);
        return state.visionPoint.dist + deviation;
    }

    public double getRawCenterX() {
        return state.visionPoint.cX;
    }

    public void setCameraEnabled(boolean cameraEnabled) {
        Camera.cameraEnabled = cameraEnabled;
        led.setCameraLed(cameraEnabled);
        socket.setEnabled(cameraEnabled);
    }

    public void setEnabled() {
        setCameraEnabled(!cameraEnabled);
    }

    public boolean checkSystem() {
        return true;
    }

    private void cachePoint() {
        // self.cx+'|'+self.cy+'|' + self.distance + "\n"
        String[] data = socket.request("point");
        if (data.length < 4) {
            System.out.println(
                "CAMERA DEBUG: Malformed point line: " + String.join("|", data)
            );
            return;
        }
        state.visionPoint.cX = Double.parseDouble(data[1]);
        state.visionPoint.cY = Double.parseDouble(data[2]);
        distances.add(Double.parseDouble(data[3]));
        state.visionPoint.deltaX = parseDeltaX(state.visionPoint.cX);

        if (loops % 5 == 0) {
            state.visionPoint.dist = distances
                .stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(0.0);
            distances.clear();
        }
    }

    public void stop() {
        socket.close();
    }

    public void readFromHardware() {
        if (socket.shouldReconnect()) {
            socket.connect();
        }
        if (socket.isConnected() && loops % 2 == 0) {
            cachePoint();
        }
        loops++;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("Dummy Camera Distance", this.shuffleBoardDistance);
        SmartDashboard
            .getEntry("Dummy Camera Distance")
            .addListener(
                notification -> setShuffleBoardDistance(notification.value.getDouble()),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
            );
    }

    public void setShuffleBoardDistance(double shuffleBoardDistance){
        this.shuffleBoardDistance = shuffleBoardDistance;
        System.out.println("setting camera dummy distance to " + shuffleBoardDistance);
    }
}
