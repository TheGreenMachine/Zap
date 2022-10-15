package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.*;

@Singleton
public class Camera extends Subsystem {

    // Components
    static LedManager led;

    private final PhotonCamera cam;
    // Constants
    private static final String NAME = "camera";
    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px
    private static final double CAMERA_HFOV = 85;
    public static final double CAMERA_VFOV = 54; // 2 * Math.atan((VIDEO_WIDTH / 2) / CAMERA_FOCAL_LENGTH); // deg
    private final double MAX_DIST = Constants.fieldCenterX; // distance of half-field
    private final double MAX_DELTA_X = 90; // degrees

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
        cam = new PhotonCamera("zed");
        SmartDashboard.putNumber("Camera/cy", 0);
    }

    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
            return simulateDeltaX();
        }
        var result = cam.getLatestResult();
        if (!result.hasTargets() || getDistance() <= -1) {
            return 0;
        }
        return result.getBestTarget().getYaw();
    }

    public double getDistance() {
        if (RobotBase.isSimulation()) {
            return robotState.getEstimatedDistanceToGoal();
        }
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return -1.0;
        }
        double distMeters = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch())
        );
        if(distMeters > MAX_DIST){
            return -1;
        }
        return Units.metersToInches(
            distMeters
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

    public void readFromHardware() {}

    public boolean checkSystem() { // this doesn't actually do anything because there's no read calls
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
