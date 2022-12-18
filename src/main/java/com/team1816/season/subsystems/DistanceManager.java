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

/**
 * A helper class that maps distance outputs to subsystem outputs through the definition of control points
 * @see com.team1816.season.states.Orchestrator
 */
@Singleton
public class DistanceManager {

    private static final RobotFactory factory = Robot.getFactory();

    /** State */
    private int lastBucketIndex;
    private static boolean allowBucketOffset = false;
    private static Spline buckets;

    /** Control Points */
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

    /**
     * Instantiates the DistanceManager and decides computational routes based on factory
     */
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

    /** Subsystem Outputs */

    /**
     * Returns the output of the spindexer based on a distance
     * @param distance
     * @return output
     * @see Spindexer
     */
    private double getSpindexerOutput(double distance) {
        return .38;
    }

    /**
     * Returns the output of the elevator based on a distance
     * @param distance
     * @return output
     * @see Elevator
     */
    private double getElevatorOutput(double distance) {
        return .50;
    }

    /**
     * Returns the output of the shooter based on a distance
     * @param distance
     * @return output
     * @see Shooter
     */
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

    /**
     * Universal get output for registered subsystems
     * @param distance
     * @param subsystem SUBSYSTEM
     * @return output
     */
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

    /** Actions */

    /**
     * Increments a specific bucket by a certain value and recomputes the path
     * @param incrementVal increment
     */
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

    /**
     * Displays knotPoint buckets on SmartDashboard / Shuffleboard
     */
    public void outputBucketOffsets() {
        for (int i = 0; i < points.size(); i++) {
            SmartDashboard.putNumber(
                "Buckets/Bucket #" + points.get(i)[0],
                points.get(i)[2]
            );
        }
    }

    /**
     * Changes the outputs displayed based on the lasBucketIndex for any changes made
     */
    public void outputCurrentBucketOffset() {
        SmartDashboard.putNumber(
            "Buckets/Bucket #" + points.get(lastBucketIndex)[0],
            points.get(lastBucketIndex)[2]
        );
    }

    /**
     * Base enum for recognized subsystems that rely on the DistanceManager
     */
    public enum SUBSYSTEM {
        SPINDEXER,
        ELEVATOR,
        SHOOTER,
    }
}
