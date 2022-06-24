package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.vision.VisionSocket;
import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class Camera extends Subsystem {

    // Components
    @Inject
    static LedManager led;

    public final VisionSocket socket = new VisionSocket();
    // Constants
    private static final String NAME = "camera";
    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px
    private static final double CAMERA_HFOV = 85;
    public static final double CAMERA_VFOV = 54; // 2 * Math.atan((VIDEO_WIDTH / 2) / CAMERA_FOCAL_LENGTH); // deg
    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 260);
    private final double MAX_DELTA_X = factory.getConstant(NAME, "maxDeltaX", 1200);

    // state
    private boolean cameraEnabled;

    public Camera() {
        super(NAME);
        SmartDashboard.putNumber("Camera/cy", 0);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    private double parseDeltaX(double x) {
        if (Math.abs(x) > MAX_DELTA_X || x < 0) {
            return 0;
        }
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        double base = Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)); // Converted to degrees
        return base;
    }

    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
            return simulateDeltaX();
        } else {
            return parseDeltaX(robotState.visionPoint.cX);
        }
    }

    public double getDistance() {
        if (RobotBase.isSimulation()) {
            return robotState.getEstimatedDistanceToGoal();
        }
        return (
            (Constants.kHeightFromCamToHub) /
            (
                Math.tan(
                    Math.toRadians(
                        Constants.kCameraMountingAngleY +
                        (
                            (
                                (VIDEO_HEIGHT - robotState.visionPoint.cY) -
                                (VIDEO_HEIGHT / 2)
                            ) *
                            CAMERA_VFOV /
                            VIDEO_HEIGHT
                        )
                    )
                )
            )
        );
    }

    public double getRawCenterX() {
        return robotState.visionPoint.cX;
    }

    public void setCameraEnabled(boolean cameraEnabled) {
        if (this.isImplemented()) {
            this.cameraEnabled = cameraEnabled;
            led.setCameraLed(cameraEnabled);
            socket.setEnabled(cameraEnabled);
        } else {
            System.out.println("not enabling camera because camera not implemented...");
        }
    }

    public boolean isEnabled() {
        return cameraEnabled;
    }

    public void toggleEnabled() {
        setCameraEnabled(!cameraEnabled);
    }

    private void cachePoint() {
        // self.cx+'|'+self.cy+'|' + self.distance + "\n"
        String[] data = socket.request("point");
        if (data == null || data.length < 4) {
            if (data != null) System.out.println("CAMERA DEBUG: Malformed point line: ");
            return;
        }
        robotState.visionPoint.cX = Double.parseDouble(data[1]);
        robotState.visionPoint.cY = Double.parseDouble(data[2]);
    }

    public void stop() {
        socket.close();
    }

    public void readFromHardware() {
        if (RobotBase.isSimulation()) {
            return;
        }
        if (socket.shouldReconnect()) {
            socket.connect();
        }
        if (socket.isConnected()) {
            cachePoint();
        }
    }

    public boolean checkSystem() { // this doesn't actually do anything because there's no read calls
        if (this.isImplemented()) {
            setCameraEnabled(true);
            Timer.delay(2);
            if (getDistance() < 0 || getDistance() > MAX_DIST) {
                System.out.println("getDistance failed test!");
                return false;
            } else if (getDeltaX() > MAX_DELTA_X) {
                System.out.println("getDeltaX failed test!");
                return false;
            }
            setCameraEnabled(false);
        }
        return true;
    }

    public double simulateDeltaX() {
        double opposite =
            Constants.fieldCenterY - robotState.getFieldToTurretPos().getY();
        double adjacent =
            Constants.fieldCenterX - robotState.getFieldToTurretPos().getX();
        double targetTurretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) {
            targetTurretAngle += Math.PI;
        }
        targetTurretAngle *= 180 / Math.PI;
        double currentTurretAngle = robotState
            .getFieldToTurretPos()
            .getRotation()
            .getDegrees();
        if (currentTurretAngle < 0 && adjacent < 0) {
            currentTurretAngle += 360;
        }
        return ((currentTurretAngle - targetTurretAngle)); //scaling for the feedback loop
    }
}
