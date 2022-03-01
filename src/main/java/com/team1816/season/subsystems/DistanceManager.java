package com.team1816.season.subsystems;

public class DistanceManager {

    // State
    private final Entry[] buckets;

    // Constants
    public DistanceManager() {
        buckets = distance_buckets;
    }

    static class Entry {

        public final double distance;
        public final double spindexerOutput;
        public final double elevatorOutput;
        public final double shooterVelocity;
        public final double hoodRetracted;

        Entry(
            double distance,
            double spindexerOutput,
            double elevatorOutput,
            double shooterVelocity,
            double hoodRetracted
        ) {
            this.distance = distance;
            this.spindexerOutput = spindexerOutput;
            this.elevatorOutput = elevatorOutput;
            this.shooterVelocity = shooterVelocity;
            this.hoodRetracted = hoodRetracted;
        }

        Entry() {
            this(0, 0, 0, 0, 0);
        }
    }

    private final Entry[] distance_buckets = new Entry[] {
        new Entry(180, 1.7, 2, 9900, 0),
        new Entry(250, 1.7, 1, 10100, 0),
        new Entry(280, 1.6, 1, 10300, 0),
        new Entry(340, 1.5, 1, 10700, 0),
    };

    private double getSpindexerOutput(double distance) {
        for (Entry velocity : buckets) {
            if (distance <= velocity.distance) {
                return velocity.spindexerOutput;
            }
        }
        return 1;
    }

    private double getElevatorOutput(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.elevatorOutput;
            }
        }
        return 1.25;
    }

    private double getShooterVelocity(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.shooterVelocity;
            }
        }
        return 0;
        //return Shooter.MAX_VELOCITY;
    }

    private double getHoodRetracted(double distance) {
        for (Entry bucket : buckets) {
            if (distance <= bucket.distance) {
                return bucket.hoodRetracted;
            }
        }
        return 0;
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
}
