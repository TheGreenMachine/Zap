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
        flatBuckets = true; // set here whether to use flat bucket values
        if (flatBuckets) {
            buckets = flat_buckets;
        } else {
            buckets = equation_buckets;
        }
        outputBucketOffsets();
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

    private final Entry[] equation_buckets = new Entry[] { //TODO: Just use the spline utility for this
        new Entry(1.40325, 0, Shooter.NEAR_VELOCITY, 0),
        new Entry(2.53272, 62.5, 1137.5, 0),
        new Entry(2.87563, 50, 2450, 0),
        new Entry(3.19722, 50, 2450, 0),
        new Entry(3.50475, 20, 6200, 0),
        new Entry(3.80236, 20, 6200, 0),
        new Entry(4.09263, 40, 3400, 0),
        new Entry(4.37730, 25, 5575.5, 0),
        new Entry(4.65756, 27.5, 5187.5, 0),
        new Entry(5.07153, 47.5, 2197.5, 0),
        new Entry(5.34110, 20, 6700, -800),
        new Entry(5.61449, 64, -1660, -600),
        new Entry(6.14997, 73.5, -3560, -700),
    };

    private final Entry[] flat_buckets = new Entry[] {
        new Entry(1.40325, 0, Shooter.NEAR_VELOCITY, 0),
        new Entry(2.53272, 0, 7600, 0),
        new Entry(2.87563, 0, 8100, 0),
        new Entry(3.19722, 0, 8800, 0),
        new Entry(3.50475, 0, 8600, 0),
        new Entry(3.80236, 0, 8700, 0),
        new Entry(4.09263, 0, 9350.5, 0),
        new Entry(4.37730, 0, 9825, 0),
        new Entry(4.65756, 0, 10210, 0),
        new Entry(5.07153, 0, 11200, 0),
        new Entry(5.34110, 0, 10240, 0),
        new Entry(5.61449, 0, 11175, 0),
        new Entry(6.14997, 0, 11875, 0),
        new Entry(6.67996, 0, 11875, 0),
        new Entry(7.33675, 0, 11875, 0),
        new Entry(7.85868, 0, 11875, 0),
        new Entry(8.50766, 0, 11875, 0),
        new Entry(9.02489, 0, 11875, 0),
        new Entry(9.54048, 0, 11875, 0),
        new Entry(10.05479, 0, 11875, 0),
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
        SmartDashboard.putNumber("Camera/Last Distance", distance);
        System.out.println("distance value too large!!");
        return Shooter.MID_VELOCITY; // this was the else statement at the end
    }

    public void incrementBucket(double incrVal) {
        if (allowBucketOffset) {
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

    public double getOutput(double distance, SUBSYSTEM subsystem) {
        switch (subsystem) {
            case SPINDEXER:
                return getSpindexerOutput(distance);
            case ELEVATOR:
                return getElevatorOutput(distance);
            case SHOOTER:
                return getShooterVelocity(distance);
        }
        System.out.println("not a SUBSYSTEM!");
        return 0;
    }

    public enum SUBSYSTEM {
        SPINDEXER,
        ELEVATOR,
        SHOOTER,
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
