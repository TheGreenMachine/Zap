package com.team1816.season.subsystems;

import com.team1816.season.util.Spline;

import java.util.ArrayList;

public class DistanceManager {

    // State
    public static ArrayList<Double[]> coordinates;
    public static ArrayList<ArrayList<Double>> coefficients = new ArrayList<>();

    // Constants
    public static ArrayList<Double[]> shooterMap = new ArrayList<>() {
        {
            add(new Double[] { 97.0, 7200.0 });
            add(new Double[] { 105.0, 7700.0 });
            add(new Double[] { 115.0, 8200.0 });
            add(new Double[] { 125.0, 8700.0 });
            add(new Double[] { 137.0, 8900.0 });
            add(new Double[] { 145.0, 9200.0 });
            add(new Double[] { 155.0, 9450.0 });
            add(new Double[] { 165.0, 9725.0 });
            add(new Double[] { 175.0, 10200.0 });
            add(new Double[] { 190.0, 10500.0 });
            add(new Double[] { 200.0, 11140.0 });
            add(new Double[] { 210.0, 11875.0 });
            add(new Double[] { 220.0, 12400.0 });
        }
    };

    public DistanceManager() {
        coordinates = shooterMap;
        calculateLinearizedSpline();
        calculateNaturalCubicSpline();
    }

    private void calculateLinearizedSpline() {
        Spline spline = new Spline(coordinates);
        coefficients = spline.calculateLinearizedSpline();
    }

    private void calculateNaturalCubicSpline() {
        Spline spline = new Spline(coordinates);
        coefficients = spline.calculateNaturalCubicSpline();
    }

    private double getSpindexerOutput(double distance) {
        return .38;
    }

    private double getElevatorOutput(double distance) { // TODO lots of hacks here - generate real outputs with hood in future
        return .5;
    }

    private double getShooterOutput(double distance) {
        for (int i = 0; i < coordinates.size() - 1; i++) {
            if (distance < coordinates.get(i + 1)[0]) { // this is because we will only generate n polynomials for n+1 points
                double output = 0;
                for (int j = 0; j < coefficients.get(i).size(); j++) {
                    output += Math.pow(distance, j) * coefficients.get(i).get(j);
                }
                return output;
            }
        }
        return coordinates.get(coordinates.size()-1)[1]; // dummy return might have to change later
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
}
