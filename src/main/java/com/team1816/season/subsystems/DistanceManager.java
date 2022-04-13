package com.team1816.season.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceManager {

    // State
    private int lastBucketIndex;
    private boolean allowBucketOffset;

    // Constants
    public DistanceManager() {
        lastBucketIndex = 0;
        allowBucketOffset = false;
        distance_buckets[lastBucketIndex].calculateAndUpdate(0);
    }

    static class Entry {

        public final double distance;
        public final double multiplier;
        public final double constant;
        public double bumpOffset;

        Entry(double distance, double multiplier, double constant, double bumpOffset) {
            this.distance = distance;
            this.multiplier = multiplier;
            this.constant = constant;
            this.bumpOffset = bumpOffset;
        }

        Entry() {
            this(0, 0, 0, 0);
        }

        public double calculateAndUpdate(double distance) {
            SmartDashboard.putNumber("Camera/Last Distance", distance);
            return multiplier * distance + constant + bumpOffset;
        }
    }

    private final Entry[] distance_buckets = new Entry[] {
        new Entry(80, 0, Shooter.NEAR_VELOCITY, 0),
        new Entry(105, 62.5, 1137.5, 0),
        new Entry(115, 50, 2450, 0),
        new Entry(125, 50, 2450, 0),
        new Entry(135, 20, 6200, 0),
        new Entry(140, 20, 6200, 0),
        new Entry(145, 40, 3400, 0),
        new Entry(155, 25, 5575.5, 0),
        new Entry(165, 27.5, 5187.5, 0),
        new Entry(175, 47.5, 2197.5, 0),
        new Entry(190, 20, 6700, 0),
        new Entry(200, 64, -1660, 0),
        new Entry(210, 73.5, -3560, 0),
    };

    private double getShooterVelocity(double distance) {
        allowBucketOffset = true;

        for (int i = 0; i < distance_buckets.length; i++) {
            Entry bucket = distance_buckets[i];
            if (distance < bucket.distance) {
                lastBucketIndex = i;
                return bucket.calculateAndUpdate(distance);
            }
        }
        return 52.5 * distance + 850; // this was the else statement at the end
    }

    public void incrementBucket(double incrVal) {
        if (allowBucketOffset) {
//            allowBucketOffset = false;
            distance_buckets[lastBucketIndex].bumpOffset += incrVal;
            System.out.println(
                "incrementing bucket #" +
                    distance_buckets[lastBucketIndex].distance +
                " offset to " +
                distance_buckets[lastBucketIndex].bumpOffset
            );
            outputCurrentBucketOffset();
        } else {
            System.out.println("not incrementing bucket...");
        }
    }

    // these are either not being called or aren't currently useful - tune later if needed
    private double getSpindexerOutput(double distance) {
        return .38;
    }

    private double getElevatorOutput(double distance) {
        return .5;
    }

    private double getHoodRetracted(double distance) {
        if (distance < 90) {
            return 0;
        }
        return 1;
    }

    public double getOutput(double distance, SUBSYSTEM subsystem) {
        switch (subsystem) {
            case SPINDEXER:
                return getSpindexerOutput(distance);
            case ELEVATOR:
                return getElevatorOutput(distance);
            case SHOOTER:
                return getShooterVelocity(distance);
            case HOOD:
                return getHoodRetracted(distance);
        }
        System.out.println("not a SUBSYSTEM!");
        return 0;
    }

    public enum SUBSYSTEM {
        SPINDEXER,
        ELEVATOR,
        SHOOTER,
        HOOD,
    }

    public void outputBucketOffsets() {
        for (int i = 0; i < distance_buckets.length; i++) {
            SmartDashboard.putNumber(
                "Buckets/Bucket #" + distance_buckets[i].distance,
                distance_buckets[i].bumpOffset
            );
        }
    }

    public void outputCurrentBucketOffset() {
        SmartDashboard.putNumber(
            "Buckets/Bucket #" + distance_buckets[lastBucketIndex].distance,
            distance_buckets[lastBucketIndex].bumpOffset
        );
    }
}
