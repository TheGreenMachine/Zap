package com.team1816.season.subsystems;

import java.util.ArrayList;

public class DistanceManager {

    // State
    public static ArrayList<Double[]> coordinates;
    public static ArrayList<ArrayList<Double>> coefficients = new ArrayList<>();

    // Constants
    public DistanceManager() {
        coordinates = shooterMap;
        calculateLinearizedSpline();
        calculateNaturalCubicSpline();
    }

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
    }; // TODO: this needs to be initialized properly

    private void calculateLinearizedSpline() {
        ArrayList<ArrayList<Double>> lCoefficients = new ArrayList<>();
        for (int i = 1; i < coordinates.size(); i++) {
            ArrayList<Double> tempCoefficients = new ArrayList<>();
            double constant =
                coordinates.get(i - 1)[1] -
                coordinates.get(i - 1)[0] *
                (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) /
                (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
            double slope =
                (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) /
                (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
            tempCoefficients.add(constant);
            tempCoefficients.add(slope);
            lCoefficients.add(tempCoefficients);
        }
        coefficients = lCoefficients;
    }

    private void calculateNaturalCubicSpline() { // this allows for continuous differentiability to the second degree and similar invertibility
        int n = coordinates.size();
        ArrayList<ArrayList<Double>> nCoefficients = new ArrayList<>();
        // creating a tridiagonal matrix equation to solve for spline coefficients
        double[] constant = new double[n - 1], linear = new double[n -
        1], quadratic = new double[n - 1], cubic = new double[n - 1]; // absolute coefficients
        double[] d = new double[n - 1], c = new double[n - 1], b = new double[n -
        1], a = new double[n - 1]; // relative coefficients d + c * (x - xi) + b * (x - xi)^2 + a * (x - xi)^3
        double[] si = new double[n], siTemp = new double[n - 2];
        si[0] = 0;
        si[n - 1] = 0;
        double[][] matrix = new double[n - 2][n - 1];
        double[] h = new double[n - 1], y = new double[n];
        for (int i = 1; i < coordinates.size(); i++) {
            h[i - 1] = coordinates.get(i)[0] - coordinates.get(i - 1)[0];
        }
        for (int i = 0; i < coordinates.size(); i++) {
            y[i] = coordinates.get(i)[1];
        }
        generateTridiagonalMatrix(n - 1, h, matrix, y);
        gaussianElimination(n - 2, n - 1, matrix, siTemp);
        for (int i = 1; i < coordinates.size() - 1; i++) {
            si[i] = siTemp[i - 1];
        }
        calculateCubicSplineOffsetedCoefficients(n - 1, h, si, y, a, b, c, d);
        for (int i = 1; i < coordinates.size(); i++) { // compression to limit floating point calculation
            ArrayList<Double> tempCoefficients = new ArrayList<>();

            constant[i - 1] =
                d[i - 1] -
                c[i - 1] *
                coordinates.get(i - 1)[0] +
                b[i - 1] *
                Math.pow(coordinates.get(i - 1)[0], 2) -
                a[i - 1] *
                Math.pow(coordinates.get(i - 1)[0], 3);
            linear[i - 1] =
                c[i - 1] -
                2 *
                b[i - 1] *
                coordinates.get(i - 1)[0] +
                3 *
                a[i - 1] *
                Math.pow(coordinates.get(i - 1)[0], 2);
            quadratic[i - 1] = b[i - 1] - 3 * a[i - 1] * coordinates.get(i - 1)[0];
            cubic[i - 1] = a[i - 1];

            tempCoefficients.add(constant[i - 1]);
            tempCoefficients.add(linear[i - 1]);
            tempCoefficients.add(quadratic[i - 1]);
            tempCoefficients.add(cubic[i - 1]);
            nCoefficients.add(tempCoefficients);
        }
        coefficients = nCoefficients;
    }

    void gaussianElimination(int m, int n, double a[][], double x[]) { // gaussian elimination for matrix manipulation
        int i, j, k;
        for (i = 0; i < m - 1; i++) {
            //Begin Gauss Elimination
            for (k = i + 1; k < m; k++) {
                double term = a[k][i] / a[i][i];
                for (j = 0; j < n; j++) {
                    a[k][j] = a[k][j] - term * a[i][j];
                }
            }
        }
        //Begin Back-substitution
        for (i = m - 1; i >= 0; i--) {
            x[i] = a[i][n - 1];
            for (j = i + 1; j < n - 1; j++) {
                x[i] = x[i] - a[i][j] * x[j];
            }
            x[i] = x[i] / a[i][i];
        }
    }

    private void calculateCubicSplineOffsetedCoefficients(
        int n,
        double h[],
        double sig[],
        double y[],
        double a[],
        double b[],
        double c[],
        double d[]
    ) { // h, a, b, c, d are of size n, sig and y are of size n+1
        int i;
        for (i = 0; i < n; i++) {
            d[i] = y[i];
            b[i] = sig[i] / 2.0;
            a[i] = (sig[i + 1] - sig[i]) / (h[i] * 6.0);
            c[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (2 * sig[i] + sig[i + 1]) / 6.0;
        }
    }

    private void generateTridiagonalMatrix(int n, double h[], double a[][], double y[]) { //h is of size n, a is of size n-1 by n, y is of size n+1
        int i;
        //int n = h.length;
        for (i = 0; i < n - 1; i++) {
            a[i][i] = 2 * (h[i] + h[i + 1]);
        }
        for (i = 0; i < n - 2; i++) {
            a[i][i + 1] = h[i + 1];
            a[i + 1][i] = h[i + 1];
        }
        for (i = 1; i < n; i++) {
            a[i - 1][n - 1] =
                (y[i + 1] - y[i]) *
                6 /
                (double) h[i] -
                (y[i] - y[i - 1]) *
                6 /
                (double) h[i - 1];
        }
    }

    private double getSpindexerOutput(double distance) {
        return .38;
    }

    private double getElevatorOutput(double distance) { // TODO lots of hacks here - generate real outputs with hood in future
        return .5;
    }

    private double getShooterVelocity(double distance) {
        if (distance < 80) {
            return Shooter.NEAR_VELOCITY;
        } else if (distance < 105) {
            return 62.5 * distance + 1137.5;
        } else if (distance < 115) {
            return 50 * distance + 2450;
        } else if (distance < 125) {
            return 50 * distance + 2450;
        } else if (distance < 135) {
            return 20 * distance + 6200;
        } else if (distance < 140) {
            return 20 * distance + 6200;
        } else if (distance < 145) {
            return 40 * distance + 3400;
        } else if (distance < 155) {
            return 25 * distance + 5575.5;
        } else if (distance < 165) {
            return 27.5 * distance + 5187.5;
        } else if (distance < 175) {
            return 47.5 * distance + 2197.5;
        } else if (distance < 190) {
            return 20 * distance + 6700;
        } else if (distance < 200) {
            return 64 * distance - 1660;
        } else if (distance < 210) {
            return 73.5 * distance - 3560;
        } else {
            return 52.5 * distance + 850;
        }
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
        return 52.5 * distance + 850; //dummy return might have to change later
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
}
