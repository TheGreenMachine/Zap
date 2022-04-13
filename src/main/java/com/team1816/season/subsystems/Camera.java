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
import edu.wpi.first.wpilibj.Timer;
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
    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    public static final double ALLOWABLE_DISTANCE_ERROR = factory.getConstant(
        NAME,
        "allowableDistanceError",
        50
    ); // deg
    //    private Queue<Double> distances = new PriorityQueue<Double>();
    private ArrayList<Double> distances = new ArrayList<>();
    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 260);
    private final double MAX_DELTA_X = factory.getConstant(NAME, "maxDeltaX", 672);

    // state
    private int loops = 0;
    private double lastDistance = 0;

    public Camera() {
        super(NAME);
        socket.setDebug(factory.getConstant(NAME, "debug") > 0);
    }

    private double parseDeltaX(double x) {
        //        if (Math.abs(x) > MAX_DELTA_X) { // ignore if x bigger than max allowed value
        //            return 0;
        //        }
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
        System.out.println("delta x = " + state.visionPoint.deltaX);
        return state.visionPoint.deltaX;
    }

    public double getDistance() {
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
        if (this.isImplemented()) {
            Camera.cameraEnabled = cameraEnabled;
            led.setCameraLed(cameraEnabled);
            socket.setEnabled(cameraEnabled);
        } else {
            System.out.println("not enabling camera because camera not implemented...");
        }
    }

    public void setEnabled() {
        setCameraEnabled(!cameraEnabled);
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
        System.out.println(state.visionPoint.cX);

        double dis = Double.parseDouble(data[3]);
        if (
            dis > 0 &&
            dis < MAX_DIST &&
            Math.abs(dis - lastDistance) < ALLOWABLE_DISTANCE_ERROR
        ) {
            distances.add(dis);
        }

        if (distances.size() > 2) { // note - this number was 5 before!
            double distanceSum = 0;
            for (int i = 0; i < distances.size(); i++) {
                distanceSum += distances.get(i);
            }
            state.visionPoint.dist = distanceSum / distances.size();
            distances.remove(0);
            state.visionPoint.deltaX = parseDeltaX(state.visionPoint.cX);
        }
        lastDistance = dis;
    }

    public void stop() {
        socket.close();
    }

    public void readFromHardware() {
        if (socket.shouldReconnect()) {
            socket.connect();
        }
        if (socket.isConnected()) {
            cachePoint();
        }
        loops++;
    }

    public boolean checkSystem() {
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
