package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import com.team1816.lib.vision.VisionSocket;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class Camera extends Subsystem {

    private static final String NAME = "camera";

    public final VisionSocket socket = new VisionSocket();

    public boolean enabled;

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

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    private double parseDeltaX(double x) {
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        double base = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
        double deviation = factory.getConstant(NAME, "deviation", 0);
        return base + deviation;
    }

    public double getDeltaXAngle() {
        return state.visionPoint.deltaX;
    }

    public double getDistance() {
        if(factory.getConstant(NAME, "useShuffleboard", 0) > 0){
            return shuffleBoardDistance;
        }

        return state.visionPoint.dist;
    }

    public double getRawCenterX() {
        return state.visionPoint.cX;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        led.setCameraLed(enabled);
        socket.setEnabled(enabled);
    }

    public void setEnabled() {
        setEnabled(!enabled);
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
        state.visionPoint.dist = Double.parseDouble(data[3]);
        state.visionPoint.deltaX = parseDeltaX(state.visionPoint.cX);
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
