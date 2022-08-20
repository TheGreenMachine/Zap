package com.team1816.season.subsystems;

import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.motion.splines.FloorFunctionSpline;
import com.team1816.lib.motion.splines.LinearPiecewiseSpline;
import com.team1816.lib.motion.splines.NaturalCubicSpline;
import com.team1816.lib.motion.splines.Spline;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import javax.inject.Singleton;

@Singleton
public class DistanceManager {

    private static final RobotFactory factory = Robot.getFactory();

    // State
    private int lastBucketIndex;
    private static boolean flatBuckets;
    private static boolean allowBucketOffset = false;
    private static Spline buckets;

    // Constants
    private static final ArrayList<Double[]> points = new ArrayList<>() { // format: {distance, output, offset}
        {
            add(new Double[] { 1.40325, (double) Shooter.NEAR_VELOCITY, 0d });
            add(new Double[] { 2.53272, 7680d, 0d });
            add(new Double[] { 2.87653, 8100d, 0d });
            add(new Double[] { 3.19722, 8800d, 0d });
            add(new Double[] { 3.50475, 8600d, 0d });
            add(new Double[] { 3.80236, 8700d, 0d });
            add(new Double[] { 4.09263, 9350.5d, 0d });
            add(new Double[] { 4.37730, 9825d, 0d });
            add(new Double[] { 4.65756, 10210d, 0d });
            add(new Double[] { 5.07153, 11200d, 0d });
            add(new Double[] { 5.34110, 10240d, 0d });
            add(new Double[] { 5.61449, 11175d, 0d });
            add(new Double[] { 6.14997, 11875d, 0d });
            add(new Double[] { 6.67996, 11875d, 0d });
            add(new Double[] { 7.33675, 11875d, 0d });
            add(new Double[] { 7.85868, 11875d, 0d });
            add(new Double[] { 8.50766, 11875d, 0d });
            add(new Double[] { 9.02489, 11875d, 0d });
            add(new Double[] { 9.54048, 11875d, 0d });
            add(new Double[] { 10.05479, 11875d, 0d });
        }
    };

    public DistanceManager() {
        lastBucketIndex = 0;
        allowBucketOffset = false;
        buckets = new FloorFunctionSpline(points);
        if (factory.getConstant("shooter", "useLinearPiecewise") == 1) {
            buckets = new LinearPiecewiseSpline(points);
        } else if (factory.getConstant("shooter", "useCubicSpline") == 1) {
            buckets = new NaturalCubicSpline(points);
        }
        outputBucketOffsets();
    }

    // these are either not being called or aren't currently useful - tune later if needed
    private double getSpindexerOutput(double distance) {
        return .38;
    }

    private double getElevatorOutput(double distance) {
        return .50;
    }

    private double getShooterOutput(double distance) {
        // used determine the last index at which any change to a certain "bucket" should be applied to
        allowBucketOffset = true;
        for (int i = 0; i < points.size(); i++) {
            if (distance < points.get(i)[0]) {
                lastBucketIndex = i;
                break;
            }
        }
        return buckets.getValue(distance);
    }

    public void incrementBucket(double incrementVal) {
        if (allowBucketOffset) {
            allowBucketOffset = false;
            points.get(lastBucketIndex)[2] += incrementVal;
            buckets = new FloorFunctionSpline(points);
            System.out.println(
                "incrementing bucket " +
                lastBucketIndex +
                " offset by " +
                points.get(lastBucketIndex)[2]
            );
            outputCurrentBucketOffset();
        } else {
            System.out.println("not incrementing bucket...");
        }
    }

    public double getOutput(double distance, SUBSYSTEM subsystem) {
        switch (subsystem) {
            case SPINDEXER:
                return getSpindexerOutput(distance);
            case ELEVATOR:
                return getElevatorOutput(distance);
            case SHOOTER:
                return getShooterOutput(distance);
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
        for (int i = 0; i < points.size(); i++) {
            SmartDashboard.putNumber(
                "Buckets/Bucket #" + points.get(i)[0],
                points.get(i)[2]
            );
        }
    }

    public void outputCurrentBucketOffset() {
        SmartDashboard.putNumber(
            "Buckets/Bucket #" + points.get(lastBucketIndex)[0],
            points.get(lastBucketIndex)[2]
        );
    }
}
