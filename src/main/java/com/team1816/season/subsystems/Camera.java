package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.visionUtil.GreenPhotonCamera;
import com.team1816.lib.util.visionUtil.GreenSimVisionSystem;
import com.team1816.lib.util.visionUtil.GreenSimVisionTarget;
import com.team1816.season.Constants;
import com.team1816.season.states.FieldConfig;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import org.photonvision.*;
import org.photonvision.targeting.*;

@Singleton
public class Camera extends Subsystem {

    // Components
    static LedManager led;

    private PhotonCamera cam;
    private GreenSimVisionSystem simVisionSystem;
    // Constants
    private static final String NAME = "camera";

    // TODO find accurate values for below vars
    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px

    // zed dfov - checked for accuracy
    private static final double CAMERA_DFOV = 100; // degrees - officially 110 degrees?

    // for debugging rn
    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 20);
    private static final double CAMERA_HFOV = 85;

    private final double CAMERA_HEIGHT_METERS = 0.7493; // meters
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(
        Constants.kTargetHeight
    );
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(
        Constants.kCameraMountingAngleY
    );
    // state
    private boolean cameraEnabled;
    private PhotonTrackedTarget bestTrackedTarget;

    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        // 2023 dep on 2022 server

        if (RobotBase.isSimulation()) {
            simVisionSystem =
                new GreenSimVisionSystem(
                    "ZED-M",
                    90,
                    60,
                    Constants.kCameraMountingAngleY,
                    new Transform2d(
                        new Translation2d(-.12065, .13335),
                        Constants.EmptyRotation
                    ), //TODO update this value
                    CAMERA_HEIGHT_METERS,
                    9000,
                    3840,
                    1080,
                    0
                );
            for (int i = 0; i <= 53; i++) {
                if (FieldConfig.aprilTags.get(i) == null) {
                    continue;
                }
                simVisionSystem.addSimVisionTarget(
                    new GreenSimVisionTarget(
                        new Pose2d(
                            FieldConfig.aprilTags.get(i).getX(),
                            FieldConfig.aprilTags.get(i).getY(),
                            FieldConfig.aprilTags.get(i).getRotation().toRotation2d()
                        ),
                        FieldConfig.aprilTags.get(i).getZ(),
                        .1651, // Estimated width & height of the AprilTag
                        .1651,
                        i
                    )
                );
            }
        }
        GreenPhotonCamera.setVersionCheckEnabled(false);
        cam = new PhotonCamera("microsoft");
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
            simVisionSystem.moveCamera(
                new Transform2d(
                    robotState.getFieldToTurretPos(),
                    robotState.fieldToVehicle
                ), // sim vision inverts this Transform when calculating robotPose
                CAMERA_HEIGHT_METERS,
                Constants.kCameraMountingAngleY
            );
            simVisionSystem.processFrame(robotState.fieldToVehicle);
            robotState.field
                .getObject("camera")
                .setPose(
                    robotState.fieldToVehicle.transformBy(
                        new Transform2d(
                            robotState.fieldToVehicle,
                            robotState.getFieldToTurretPos()
                        )
                    )
                );
        }
        robotState.visibleTargets = getPoints();
    }

    public ArrayList<Point> getPoints() {
        ArrayList<Point> targets = new ArrayList<>();
        var result = cam.getLatestResult();

        if (!result.hasTargets()) {
            if (result == null) {
                System.out.println("Pipeline null");
                if (result.targets == null) {
                    System.out.println("Pipeline targets null");
                    if (result.targets.size() == 0) {
                        System.out.println("Pipeline Empty");
                    }
                }
            }
            return targets;
        }


        double m = 0xFFFFFF; // big number
        var principal_RANSAC = new PhotonTrackedTarget();

        for (PhotonTrackedTarget target : List.of(result.getBestTarget())) { //TODO reimplement result.targets instead
            var p = new Point();
            if (target.getCameraToTarget() != null) {
                p.cameraToTarget = target.getCameraToTarget();
                p.id = target.getFiducialId();
                targets.add(p);

                if (m > p.cameraToTarget.getTranslation().getNorm()) {
                    m = p.cameraToTarget.getTranslation().getNorm();
                    principal_RANSAC = target;
                }
            }
        }
        bestTrackedTarget = principal_RANSAC;

        return targets;
    }

    public double getDistance() {
        return robotState.getDistanceToGoal();
    }

    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
            return simulateDeltaX();
        }
        if (bestTrackedTarget == null) {
            getPoints();
        }
        return bestTrackedTarget.getYaw();
    }

    public boolean checkSystem() {
        if (this.isImplemented()) {
            setCameraEnabled(true);
            Timer.delay(2);
            if (getDistance() < 0 || getDistance() > MAX_DIST) {
                System.out.println("getDistance failed test!");
                return false;
            } else if (
                getDeltaX() < -CAMERA_HFOV / 2d || getDeltaX() > CAMERA_HFOV / 2d
            ) {
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
        return ((currentTurretAngle - targetTurretAngle)); // scaling for the feedback loop
    }

    /** Camera state */
    public static class Point {

        public int id; // -2 if not detected
        public Transform3d cameraToTarget;
        public double weight;

        public Point() {
            id = 0;
            cameraToTarget = new Transform3d();
            weight = 0;
        }

        public String toString() {
            return "id: " + id + " camToTar: " + cameraToTarget;
        }
    }
}
