package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

@Singleton
public class Camera {

    private static Camera INSTANCE;

    // Components
    private final NetworkTable networkTable;

    @Inject
    private static LedManager led;

    // State
    private double deltaXAngle;
    private double distance;
    private final NetworkTableEntry usingVision;

    private double rawCenterX;

    // Constants
    private static final double CAMERA_FOV = 87.0; // deg
    private static final double CAMERA_FOCAL_LENGTH = 350; // px
    private static final double VIDEO_WIDTH = 672.0; // px
    public static final double ALLOWABLE_AIM_ERROR = 1; // deg

    public Camera() {
        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        usingVision = networkTable.getSubTable("Calibration").getEntry("VISION");
        networkTable.addEntryListener(
            "center_x",
            (table, key, entry, value, flags) -> {
                rawCenterX = value.getDouble();
                if (value.getDouble() < 0) {
                    // Reset deltaX to 0 if contour not detected
                    deltaXAngle = 0;
                    return;
                }
                var deltaXPixels = (value.getDouble() - (VIDEO_WIDTH / 2)); // Calculate deltaX from center of screen
                this.deltaXAngle =
                    Math.toDegrees(Math.atan2(deltaXPixels, CAMERA_FOCAL_LENGTH)) * 0.64;
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        );

        networkTable.addEntryListener(
            "distance",
            (table, key, entry, value, flags) -> {
                // Use most recently available distance if distance not found
                this.distance = value.getDouble();
            },
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
        );
    }

    public double getDeltaXAngle() {
        return deltaXAngle;
    }

    public double getDistance() {
        return distance;
    }

    public double getRawCenterX() {
        return rawCenterX;
    }

    public void setEnabled(boolean enabled) {
        led.setCameraLed(enabled);
        usingVision.setBoolean(enabled);
    }
}
