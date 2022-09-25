package com.team1816.lib.math.motion.splines;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public abstract class Spline {

    public static ArrayList<Double[]> coordinates;
    public static ArrayList<ArrayList<Double>> coefficients;

    protected Spline(ArrayList<Double[]> knotPoints) {
        coordinates = sort(knotPoints); // properly formats and checks coordinates
    }

    public double getValue(double input) {
        for (int i = 0; i < coordinates.size() - 1; i++) {
            if (input < coordinates.get(i + 1)[0]) { // this is because we want the value to hold until the next value
                double output = coordinates.get(i).length > 2 ? coordinates.get(i)[2] : 0; // use offsets if they exist
                for (int j = 0; j < coefficients.get(i).size(); j++) {
                    output += Math.pow(input, j) * coefficients.get(i).get(j);
                }
                return output;
            }
        }
        return coordinates.get(coordinates.size() - 1)[1];
    }

    public abstract ArrayList<ArrayList<Double>> generateCoefficients();

    protected ArrayList<Double[]> sort(ArrayList<Double[]> points) { // formats the knotPoints to avoid exceptions, if done optimally O(nLog(n)) and given the minuscule number of points, it's acceptable
        ArrayList<Double[]> sorted = new ArrayList<>();
        HashMap<Double, Double[]> map = new HashMap<>();
        for (Double[] point : points) {
            if (point.length >= 2) { // ensures no malformed coordinates will be used
                map.put(point[0], point);
            }
        }
        List<Double> keys = new ArrayList<Double>(map.keySet());
        Collections.sort(keys);
        for (int i = 0; i < points.size(); i++) {
            sorted.add(map.get(keys.get(i)));
        }
        return sorted;
    }
}
