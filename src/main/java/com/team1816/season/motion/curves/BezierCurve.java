package com.team1816.season.motion.curves;

import com.team1816.season.motion.splines.NaturalCubicSpline;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * TODO: NOTE THIS CLASS IS STILL IN PROGRESS
 */

public class BezierCurve {
    public static class ControlPoint {
        private double x;
        private double y;
        public ControlPoint() {
            x = 0;
            y = 0;
        }
        public ControlPoint(double a, double b) {
            x = a;
            y = b;
        }
        public void add(ControlPoint c) {
            x+=c.x;
            y+=c.x;
        }
        public void multiply(double z) {
            x*=z;
            y*=z;
        }
    }
    private ArrayList<ControlPoint> controlPoints; // defined sequentially
    private ArrayList<Double> xCoefficients; // defined such that index refers to exponent
    private ArrayList<Double> yCoefficients; // defined such that index refers to exponent
    private NaturalCubicSpline LUT;
    public BezierCurve() {
        controlPoints = new ArrayList<>();
        xCoefficients = yCoefficients = new ArrayList<>();
    }

    public BezierCurve(ArrayList<ControlPoint> arr) {
        controlPoints = arr;
    }

    public void generateLookUpTable(int resolution) {

    }

    private ControlPoint lerp(ControlPoint p1, ControlPoint p2, double t) {
        p1.multiply(1-t);
        p2.multiply(t);
        p1.add(p2);
        return p1;
    }

    public double getValue(double t) {
        return 0;
    }

    public double getLength() {
        return 0;
    }

}
