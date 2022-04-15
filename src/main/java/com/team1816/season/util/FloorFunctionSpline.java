package com.team1816.season.util;

import java.util.ArrayList;

public class FloorFunctionSpline extends Spline {

    public static ArrayList<Double[]> coordinates;
    private static ArrayList<ArrayList<Double>> coefficients;

    public FloorFunctionSpline(ArrayList<Double[]> knotPoints) {
        super(knotPoints);
        coefficients = generateCoefficients();
    }

    @Override
    public ArrayList<ArrayList<Double>> generateCoefficients() {
        ArrayList<ArrayList<Double>> fCoefficients = new ArrayList<>();
        for (int i = 0; i < coordinates.size(); i++) {
            ArrayList<Double> tempCoefficients = new ArrayList<>();
            tempCoefficients.add(coordinates.get(i)[1]);
        }
        return fCoefficients;
    }
}
