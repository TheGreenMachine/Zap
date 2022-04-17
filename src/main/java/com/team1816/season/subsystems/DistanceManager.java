package com.team1816.season.subsystems;

import com.google.inject.Singleton;
import com.team1816.season.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

@Singleton
public class DistanceManager {

    // State
    public ArrayList<Double[]> coordinates;
    public Spline shooterOutput;

    private int lastBucketIndex;
    private boolean allowBucketOffset;

    // Constants
    public static ArrayList<Double[]> shooterMap = new ArrayList<>() { // format: {distance, output, offset}
        {
            add(new Double[] { 97.0, 7200.0, 0.0 });
            add(new Double[] { 105.0, 7700.0, 0.0 });
            add(new Double[] { 115.0, 8200.0, 0.0 });
            add(new Double[] { 125.0, 8700.0, 0.0 });
            add(new Double[] { 137.0, 8900.0, 0.0 });
            add(new Double[] { 145.0, 9200.0, 0.0 });
            add(new Double[] { 155.0, 9450.0, 0.0 });
            add(new Double[] { 165.0, 9725.0, 0.0 });
            add(new Double[] { 175.0, 10200.0, 0.0 });
            add(new Double[] { 190.0, 10500.0, 0.0 });
            add(new Double[] { 200.0, 11140.0, 0.0 });
            add(new Double[] { 210.0, 11875.0, 0.0 });
            add(new Double[] { 220.0, 12400.0, 0.0 });
        }
    };

    public DistanceManager() {
        coordinates = shooterMap;
        lastBucketIndex = 0;
        allowBucketOffset = false;
        calculateFloorFunctionSpline();
        calculateLinearizedSpline();
        calculateNaturalCubicSpline();
        calculateQuinticSpline();
    }

    private void calculateFloorFunctionSpline() {
        shooterOutput = new FloorFunctionSpline(coordinates);
    }

    private void calculateLinearizedSpline() {
        shooterOutput = new LinearPiecewiseSpline(coordinates);
    }

    private void calculateNaturalCubicSpline() {
        shooterOutput = new NaturalCubicSpline(coordinates);
    }

    private void calculateQuinticSpline() {
        shooterOutput = new QuinticSpline(coordinates);
    }

    private double getSpindexerOutput(double distance) {
        return .38;
    }

    private double getElevatorOutput(double distance) { // TODO lots of hacks here - generate real outputs with hood in future
        return .5;
    }

    private double getShooterOutput(double distance) {
        allowBucketOffset = true;
        for (int i = 0; i < coordinates.size(); i++) {
            if (distance < coordinates.get(i)[0]) {
                lastBucketIndex = i;
                break;
            }
        }
        return shooterOutput.getValue(distance);
    }

    public void incrementBucket(double incrementVal) {
        if (allowBucketOffset) {
            allowBucketOffset = false;
            coordinates.get(lastBucketIndex)[2] += incrementVal;
            System.out.println(
                "incrementing bucket " +
                lastBucketIndex +
                " offset by " +
                coordinates.get(lastBucketIndex)[2]
            );
            outputCurrentBucketOffset();
        } else {
            System.out.println("not incrementing bucket...");
        }
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
                return getShooterOutput(distance);
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
        for (int i = 0; i < coordinates.size(); i++) {
            SmartDashboard.putNumber(
                "Buckets/Bucket #" + coordinates.get(i)[0],
                coordinates.get(i)[2]
            );
        }
    }

    public void outputCurrentBucketOffset() {
        SmartDashboard.putNumber(
            "Buckets/Bucket #" + coordinates.get(lastBucketIndex)[0],
            coordinates.get(lastBucketIndex)[2]
        );
    }
}
