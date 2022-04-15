package com.team1816.season.util;

import java.util.ArrayList;

public class LinearPiecewiseSpline extends Spline {

    public static ArrayList<Double[]> coordinates;
    private static ArrayList<ArrayList<Double>> coefficients;

    public LinearPiecewiseSpline(ArrayList<Double[]> knotPoints) {
        super(knotPoints);
        coordinates = sort(knotPoints);
        coefficients = generateCoefficients();
    }

    @Override
    public ArrayList<ArrayList<Double>> generateCoefficients() {
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
}
