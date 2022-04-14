package com.team1816.season.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

@Singleton
public class DistanceManager {

    // State
    private int lastBucketIndex;
    private static boolean flatBuckets;
    private static boolean allowBucketOffset = false;
    private Entry[] buckets;

    // Constants
    public DistanceManager() {
        lastBucketIndex = 0;
        flatBuckets = true;
        //        allowBucketOffset = false;
        if (flatBuckets) {
            buckets = flat_buckets;
        } else {
            buckets = equation_buckets;
        }
        buckets[lastBucketIndex].calculateAndUpdate(0);
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

        public double calculateAndUpdate(double camDistance) {
            SmartDashboard.putNumber("Camera/Last Distance", camDistance);
            return multiplier * distance + constant + bumpOffset;
        }
    }

    private final Entry[] equation_buckets = new Entry[] {
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
        new Entry(190, 20, 6700, -800),
        new Entry(200, 64, -1660, -600),
        new Entry(210, 73.5, -3560, -700),
    };

    private final Entry[] flat_buckets = new Entry[] {
        new Entry(80, 0, Shooter.NEAR_VELOCITY, 0),
        new Entry(105, 0, 7700, 0),
        new Entry(115, 0, 7900, 0),
        new Entry(125, 0, 8300, 0),
        new Entry(135, 0, 8300, 0),
        new Entry(140, 0, 8600, 0),
        new Entry(145, 0, 9200, 0),
        new Entry(155, 0, 9450.5, 0),
        new Entry(165, 0, 9425, 0),
        new Entry(175, 0, 9710, 0),
        new Entry(190, 0, 10000, 0),
        new Entry(200, 0, 10240, 0),
        new Entry(210, 0, 11175, 0),
        new Entry(250, 0, 11875, 0),
    };

    private double getShooterVelocity(double distance) {
        allowBucketOffset = true;

        for (int i = 0; i < buckets.length; i++) {
            Entry bucket = buckets[i];
            if (distance < bucket.distance) {
                lastBucketIndex = i;
                System.out.println("last bucket index = " + lastBucketIndex);
                return bucket.calculateAndUpdate(distance);
            }
        }
        return 52.5 * distance + 850; // this was the else statement at the end
    }

    public void incrementBucket(double incrVal) {
        if (allowBucketOffset) {
            allowBucketOffset = false;
            buckets[lastBucketIndex].bumpOffset += incrVal;
            System.out.println(
                "incrementing bucket #" +
                buckets[lastBucketIndex].distance +
                " offset to " +
                buckets[lastBucketIndex].bumpOffset
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
        for (int i = 0; i < buckets.length; i++) {
            SmartDashboard.putNumber(
                "Buckets/Bucket #" + buckets[i].distance,
                buckets[i].bumpOffset
            );
        }
    }

    public void outputCurrentBucketOffset() {
        SmartDashboard.putNumber(
            "Buckets/Bucket #" + buckets[lastBucketIndex].distance,
            buckets[lastBucketIndex].bumpOffset
        );
    }
}
