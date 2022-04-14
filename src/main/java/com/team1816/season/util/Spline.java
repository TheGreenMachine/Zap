package com.team1816.season.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.HashMap;

public class Spline {
    public static ArrayList<Double[]> coordinates;

    public Spline(ArrayList<Double[]> knotPoints) {
        coordinates = sort(knotPoints);
    }

    public ArrayList<ArrayList<Double>> calculateLinearizedSpline() {
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
        return lCoefficients;
    }

    public ArrayList<ArrayList<Double>> calculateNaturalCubicSpline() { // this allows for continuous differentiability to the second degree and similar invertibility
        int n = coordinates.size();
        ArrayList<ArrayList<Double>> nCoefficients = new ArrayList<>();
        // creating a tri-diagonal matrix equation to solve for spline coefficients
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
            // converting to ax^3+bx^2+cx+d form
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
        return nCoefficients;
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

    private void generateTridiagonalMatrix(int n, double h[], double a[][], double y[]) { // h is of size n, a is of size n-1 by n, y is of size n+1
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

    private ArrayList<Double[]> sort(ArrayList<Double[]> points) { // avoid edge cases, if done optimally O(nLog(n)) and given the minuscule number of points, it's acceptable
        ArrayList<Double[]> sorted = new ArrayList<>();
        HashMap<Double, Double[]> map = new HashMap<>();
        for (Double[] point : points) {
            map.put(point[0], point);
        }
        List<Double> keys = (List<Double>) map.keySet();
        Collections.sort(keys);
        for (int i = 0; i < points.size(); i++) {
            sorted.add(map.get(keys.get(i)));
        }
        return sorted;
    }
}
