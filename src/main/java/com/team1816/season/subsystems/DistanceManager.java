package com.team1816.season.subsystems;

import java.util.ArrayList;

public class DistanceManager {

    // State
    private final Entry[] buckets;

    // Constants
    public DistanceManager() {
        buckets = distance_buckets;
    }

    public static ArrayList<Double[]> coordinates = new ArrayList<>();
    public static ArrayList<ArrayList<Double>> coefficients = new ArrayList<>();

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

    private final Entry[] distance_buckets = new Entry[]{
        new Entry(133, .85, 1, 10700, 0),
        new Entry(250, 1.7, 1, 10100, 0),
        new Entry(280, 1.6, 1, 10300, 0),
        new Entry(340, 1.5, 1, 10700, 0),
    };

    private void calculateLinearizedSpline() {
        ArrayList<ArrayList<Double>> lCoefficients = new ArrayList<>();
        for (int i = 1; i < coordinates.size(); i++) {
            ArrayList<Double> tempCoefficients = new ArrayList<>();
            double constant = coordinates.get(i - 1)[1] - coordinates.get(i - 1)[0] * (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) / (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
            double slope = (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) / (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
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
        double[] constant = new double[n - 1], linear = new double[n - 1], quadratic = new double[n - 1], cubic = new double[n - 1]; // absolute coefficients
        double[] d = new double[n - 1], c = new double[n - 1], b = new double[n - 1], a = new double[n - 1]; // relative coefficients d + c * (x - xi) + b * (x - xi)^2 + a * (x - xi)^3
        double[] si = new double[n], siTemp = new double[n - 2];
        si[0] = 0;
        si[n] = 0;
        double[][] matrix = new double[n - 2][n - 1];
        double[] h = new double[n - 1], y = new double[n];
        for (int i = 1; i < coordinates.size(); i++) {
            h[i] = coordinates.get(i)[0] - coordinates.get(i - 1)[0];
        }
        for (int i = 0; i < coordinates.size(); i++) {
            y[i] = coordinates.get(i)[1];
        }
        generateTridiagonalMatrix(n - 1, h, matrix, y);
        gaussianElimination(n - 2, n - 1, matrix, siTemp);
        for (int i = 1; i < coordinates.size() - 1; i++) {
            si[i] = siTemp[i - 1];
        }
        cubicSplineCalculate(n - 1, h, si, y, a, b, c, d);
        for (int i = 1; i < coordinates.size(); i++) { // compression to limit floating point calculation
            ArrayList<Double> tempCoefficients = new ArrayList<>();

            constant[i - 1] = d[i - 1] - c[i - 1] * coordinates.get(i - 1)[0] + b[i - 1] * Math.pow(coordinates.get(i - 1)[0], 2) - c[i - 1] * Math.pow(coordinates.get(i - 1)[0], 3);
            linear[i - 1] = c[i - 1] - 2 * b[i - 1] * coordinates.get(i - 1)[0] + 3 * a[i - 1] * Math.pow(coordinates.get(i - 1)[0], 2);
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

    private void cubicSplineCalculate(int n, double h[], double sig[], double y[], double a[], double b[], double c[], double d[]) { // h, a, b, c, d are of size n, sig and y are of size n+1
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
            a[i - 1][n - 1] = (y[i + 1] - y[i]) * 6 / (double) h[i] - (y[i] - y[i - 1]) * 6 / (double) h[i - 1];
        }
    }

    private double getSpindexerOutput(double distance) {
        //        if(distance < 110){
        //            return 0;
        //        }
        return .38;
    }

    private double getElevatorOutput(double distance) { // TODO lots of hacks here - generate real outputs with hood in future
        //        if(distance < 100){
        //            return 0.5;
        //        }
        //        for (Entry bucket : buckets) {
        //            if (distance <= bucket.distance) {
        //                return bucket.elevatorOutput;
        //            }
        //        }
        return .5;
    }

    private double getShooterVelocity(double distance) {
        //        if(distance < 90) {
        //            return Shooter.NEAR_VELOCITY;
        //        } else if(distance < 105) {
        //            return -0.05460540718377771 * Math.pow(distance,3) + 15.890173490479313 * Math.pow(distance, 2) - 1475.3520825167316 * distance + 50635.39042284306;
        //        } else if(distance < 115) {
        //            return 0.07594789843630197 * Math.pow(distance, 3) - 25.234117779845786 * Math.pow(distance, 2) + 2842.6985008674037 * distance - 100496.37999560167;
        //        } else if(distance < 125) {
        //            return -0.11763353769937684* Math.pow(distance, 3) + 41.5514776869634 * Math.pow(distance, 2) - 4837.644977815653 * distance + 193916.78668724882;
        //        } else if(distance < 135){
        //            return 0.0945862523612054* Math.pow(distance, 3) - 38.03094358575493* Math.pow(distance, 2) + 5110.15768127414 * distance - 220574.99077482586;
        //        } else if(distance < 140){
        //            return 0.21245183712487117 * Math.pow(distance, 3) - 85.76650541503957 * Math.pow(distance, 2) + 11554.458528227566* distance -510568.52888773003;
        //        } else if(distance < 145){
        //            return -0.37285463383765616* Math.pow(distance, 3) + 160.0622123892219* Math.pow(distance, 2) - 22861.56196436904* distance + 1095512.427433445;
        //        } else if(distance < 155){
        //            return 0.07621934106436455* Math.pow(distance, 3) -35.2849666931571* Math.pow(distance, 2) + 5463.779002575915 * distance - 273545.7193022279;
        //        } else if(distance < 165){
        //            return 0.06981396070988204* Math.pow(distance, 3) - 32.30646482832273* Math.pow(distance, 2) + 5002.111213526588 * distance - 249692.88353467933;
        //        } else if(distance < 175){
        //            return -0.18047518390389267* Math.pow(distance, 3) + 91.58666175549574* Math.pow(distance, 2) - 15440.254672803461* distance + 874637.2402134733;
        //        } else if(distance < 190){
        //            return 0.14898989538137067* Math.pow(distance, 3) - 81.3825048692675 * Math.pow(distance, 2) + 14829.349486530107 * distance - 891089.6690809848;
        //        } else if(distance < 200){
        //            return -0.11020660992035931* Math.pow(distance, 3) + 66.35950315271859 * Math.pow(distance, 2) - 13241.632037647249* distance + 886739.1607835811;
        //        } else if(distance < 210){
        //            return -0.06231405002095808* Math.pow(distance, 3) + 37.62396721307784* Math.pow(distance, 2) - 7494.5248497191005 * distance + 503598.68158837117;
        //        } else {
        //            return 0.05446281000419161* Math.pow(distance, 3) - 35.94545460276647* Math.pow(distance, 2) + 7955.053731608204* distance - 577871.81910454;
        //        }

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

    private double getShooterOutputAlt(double distance) {
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
