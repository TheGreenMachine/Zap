package com.team1816.frcSeason.subsystems;

import com.google.inject.Inject;

public class DistanceManager {

    private static DistanceManager INSTANCE;

    // Components
    @Inject
    private static Camera camera;

    // State
    private final Entry[] buckets;
    private int zone;

    // Constants
    public DistanceManager() {
        buckets = distance_buckets;
    }

    static class Entry {

        public final double distance;
        public final double shooterVelocity;
        public final double spindexerOutput;
        public final double turretBias;
        public final boolean hoodRetracted;

        Entry(
            double distance,
            double shooterVelocity,
            double spindexerOutput,
            double turretBias,
            boolean hoodRetracted
        ) {
            this.distance = distance;
            this.shooterVelocity = shooterVelocity;
            this.spindexerOutput = spindexerOutput;
            this.turretBias = turretBias;
            this.hoodRetracted = hoodRetracted;
        }

        Entry() {
            this(0, 0, 0, 0, false);
        }
    }

    private final Entry[] distance_buckets = new Entry[] {
        //        new Entry(167, 11_100, 2, 1.5), // untested
        //        new Entry(198, 10_400, 1, 1.5),
        //        new Entry(223, 8_300, 1, 1.45), //untested
        //        new Entry(250, 9_200, 1, 1.4),
        //        new Entry(285, 9_800, 1, 1.35),
        //        new Entry(315, 10_200, 1, 1.25),
        //        new Entry(360, 10_400, 1, 1.25),
        //        new Entry(400, 10_600, 1, 1.25)\
        new Entry(180, 10700, 2, 1.7, false),
        new Entry(250, 10100, 1, 1.7, false),
        new Entry(280, 10300, 1, 1.6, false),
        new Entry(340, 10700, 1, 1.5, false),
    };

    public double getShooterVelocity(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.shooterVelocity;
            }
        }
        return 0;
        //return Shooter.MAX_VELOCITY;
    }

    public double getShooterVelocity() {
        return getShooterVelocity(camera.getDistance());
    }

    public double getTurretBias(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.turretBias;
            }
        }
        return 1.25;
    }

    public double getTurretBias() {
        return getTurretBias(camera.getDistance());
    }

    public double getSpindexerOutput(double distance) {
        for (Entry velocity : buckets) {
            if (distance <= velocity.distance) {
                return velocity.spindexerOutput;
            }
        }
        return 1;
    }

    public double getSpindexerOutput() {
        return getSpindexerOutput(camera.getDistance());
    }

    public boolean getHoodRetracted(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.hoodRetracted;
            }
        }
        return false;
    }

    public boolean getHoodRetracted() {
        return getHoodRetracted(camera.getDistance());
    }
}
