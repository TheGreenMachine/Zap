package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.RobotState.Point;
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
    private final double kMaxDist = factory.getConstant(NAME, "maxDist", 260);
    private final double kMaxDeltaX = factory.getConstant(NAME, "maxDeltaX", 1200);

    private final int kMaxLoopCount = (int) factory.getConstant(NAME, "maxLoopCount", 0);

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
    private double checkedDist;
    private double checkedDeltaX;
    // used to make sure that if the camera intermittently sees the target (detection drop)
    // we won't get spasms between -1 and the actual values
    private int loop;

    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        SmartDashboard.putNumber("Camera/cy", 0);
    }

    public double getDeltaX() {
        return checkedDeltaX;
    }

    public double getDistance() {
        return checkedDist;
    }

    public void setCameraEnabled(boolean cameraEnabled) {
        if (isImplemented()) {
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
        if (cameraEnabled) {
            if (RobotBase.isSimulation()) { //simulate feedback loop
                checkedDeltaX = simulateDeltaX();
                checkedDist = robotState.getDistanceToGoal();
            } else {
                var result = cam.getLatestResult();
                if (!result.hasTargets()) {
                    loop++;
                    if (loop > kMaxLoopCount) {
                        checkedDeltaX = -1.0;
                        checkedDist = -1.0;
                        robotState.visionPoint = new ArrayList<Point>();
                    }
                } else {
                    loop = 0;
                    checkedDeltaX = result.getBestTarget().getYaw();
                    checkedDist =
                        PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch())
                        );
                }
                robotState.visionPoint = getPoints(result);
            }
        }
    }

    public ArrayList<RobotState.Point> getPoints(PhotonPipelineResult result) {
        var list = new ArrayList<Point>();
        for (PhotonTrackedTarget target : result.targets) {
            var p = new Point();
            var camToTarget = new Pose2d();
            camToTarget.plus(target.getCameraToTarget());
            System.out.println("need to fix getPoints method");
            //            p.id = target.getFiducialId();
            p.x = (int) camToTarget.getX();
            p.y = (int) camToTarget.getY();
            //            p.z = (int) camToTarget.getZ();
            list.add(p);
        }
        return list;
    }

    public boolean checkSystem() { // this doesn't actually do anything because there's no read calls
        if (this.isImplemented()) {
            setCameraEnabled(true);
            Timer.delay(2);
            if (getDistance() < 0 || getDistance() > kMaxDist) {
                System.out.println("getDistance failed test!");
                return false;
            } else if (getDeltaX() > kMaxDeltaX) {
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
