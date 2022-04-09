package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.vision.VisionSocket;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

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
    public static final double ALLOWABLE_AIM_ERROR = 1; // deg
    private int loops = 0;
    //    private Queue<Double> distances = new PriorityQueue<Double>();
    private ArrayList<Double> distances = new ArrayList<>();

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    private double parseDeltaX(double x) {
        if (Math.abs(x) > 672) { // ignore if x bigger than max allowed value
            return 0;
        }
        double deltaXPixels = (x - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
        double base =
            Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
        return base;
    }

    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
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
            return (.5 * (currentTurretAngle - targetTurretAngle)); //scaling for the feedback loop
        }
        return state.visionPoint.deltaX;
    }

    public double getDistance() {
        if (RobotBase.isSimulation()) {
            return robotState.getEstimatedDistanceToGoal();
        }
        if (factory.getConstant(NAME, "useShuffleboard", 0) > 0) {
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
        if (data == null || data.length < 4) {
            System.out.println("CAMERA DEBUG: Malformed point line: ");
            return;
        }
        state.visionPoint.cX = Double.parseDouble(data[1]);
        state.visionPoint.cY = Double.parseDouble(data[2]);

        double dis = Double.parseDouble(data[3]);
        if (dis > 0) {
            distances.add(dis);
        }

        if (distances.size() > 5) {
            double distanceSum = 0;
            for (int i = 0; i < 6; i++) {
                distanceSum += distances.get(i);
            }
            state.visionPoint.dist = distanceSum / distances.size();
            distances.remove(0);
            state.visionPoint.deltaX = parseDeltaX(state.visionPoint.cX);
        }
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

    public void setShuffleBoardDistance(double shuffleBoardDistance) {
        this.shuffleBoardDistance = shuffleBoardDistance;
        System.out.println("setting camera dummy distance to " + shuffleBoardDistance);
    }
}
