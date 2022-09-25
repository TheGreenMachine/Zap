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

/** A helper class that maps distance outputs to subsystem outputs through the definition of control points */
@Singleton
public class DistanceManager {

    private static final RobotFactory factory = Robot.getFactory();

    // State
    private int lastBucketIndex;
    private static boolean allowBucketOffset = false;
    private static Spline buckets;

    /** Control Points */
    private static final ArrayList<Double[]> points = new ArrayList<>() { // format: {distance, output, offset}
        {
            add(new Double[] { 80d, (double) Shooter.NEAR_VELOCITY, 0d });
            add(new Double[] { 105d, 7680d, 0d });
            add(new Double[] { 115d, 8100d, 0d });
            add(new Double[] { 125d, 8800d, 0d });
            add(new Double[] { 135d, 8600d, 0d });
            add(new Double[] { 145d, 8700d, 0d });
            add(new Double[] { 155d, 9350.5d, 0d });
            add(new Double[] { 165d, 9825d, 0d });
            add(new Double[] { 175d, 10210d, 0d });
            add(new Double[] { 190d, 11200d, 0d });
            add(new Double[] { 200d, 10240d, 0d });
            add(new Double[] { 210d, 11175d, 0d });
            add(new Double[] { 230d, 11875d, 0d });
            add(new Double[] { 250d, 11875d, 0d });
            add(new Double[] { 275d, 11875d, 0d });
            add(new Double[] { 295d, 11875d, 0d });
            add(new Double[] { 320d, 11875d, 0d });
            add(new Double[] { 340d, 11875d, 0d });
            add(new Double[] { 360d, 11875d, 0d });
            add(new Double[] { 380d, 11875d, 0d });
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

    /** subsystem outputs */
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

    /** actions */
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

    /** Smart Dashboard */
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

    /** subsystems */
    public enum SUBSYSTEM {
        SPINDEXER,
        ELEVATOR,
        SHOOTER,
        HOOD,
    }
}
