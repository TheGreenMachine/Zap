package com.team1816.season.subsystems;

import static com.team1816.season.states.RobotState.Point;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import org.photonvision.*;
import org.photonvision.targeting.*;

@Singleton
public class Camera extends Subsystem {

    // Components
    static LedManager led;

    private final PhotonCamera cam = new PhotonCamera("zed");
    // Constants
    private static final String NAME = "camera";
    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px
    private static final double CAMERA_HFOV = 85;
    public static final double CAMERA_VFOV = 54; // 2 * Math.atan((VIDEO_WIDTH / 2) / CAMERA_FOCAL_LENGTH); // deg
    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 260);
    private final double MAX_DELTA_X = factory.getConstant(NAME, "maxDeltaX", 1200);

    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(
        Constants.kCameraMountingHeight
    );
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(
        Constants.kTargetHeight
    );
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(
        Constants.kCameraMountingAngleY
    );
    // state
    private boolean cameraEnabled;

    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        SmartDashboard.putNumber("Camera/cy", 0);
    }

    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
            return simulateDeltaX();
        }
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return -1.0;
        }
        return result.getBestTarget().getPitch();
    }

    public double getDistance() {
        if (RobotBase.isSimulation()) {
            return robotState.getEstimatedDistanceToGoal();
        }
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return -1.0;
        }
        return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch())
        );
    }

    public void setCameraEnabled(boolean cameraEnabled) {
        if (this.isImplemented()) {
            this.cameraEnabled = cameraEnabled;
            led.setCameraLed(cameraEnabled);
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

    public void stop() {}

    public void readFromHardware() {
        if (RobotBase.isSimulation()) {
            return;
        }
    }

    public ArrayList<Point> getPoints() {
        ArrayList<Point> list = new ArrayList<Point>();
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return list;
        }

        for (PhotonTrackedTarget target : result.targets) {
            var p = new Point();
            var camToTarget = new Pose2d();
            camToTarget.plus(target.getCameraToTarget());
            p.id = target.getFiducialId();
            p.x = (int) camToTarget.getX();
            p.y = (int) camToTarget.getY();
            p.z = (int) camToTarget.getZ();
            list.add(p);
        }
        return list;
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
