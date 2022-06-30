package com.team1816.season.motion.curves;

import com.team1816.season.motion.splines.NaturalCubicSpline;
import java.util.ArrayList;

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
            x += c.x;
            y += c.x;
        }

        public void multiply(double z) {
            x *= z;
            y *= z;
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

    public void generateLookUpTable(int resolution) {}

    private ControlPoint lerp(ControlPoint p1, ControlPoint p2, double t) {
        p1.multiply(1 - t);
        p2.multiply(t);
        p1.add(p2);
        return p1;
    }

    public ControlPoint getValue(double t) {
        ControlPoint val = new ControlPoint();
        for(int i = 0; i <= controlPoints.size()-1; i++) {
            ControlPoint c = controlPoints.get(i);
            c.multiply(Math.pow(t, i)*Math.pow((1-t), controlPoints.size()-i-1)*combination(controlPoints.size()-1, i));
            val.add(c);
        }
        return val;
    }

    public double combination(int n, int x) {
        double ans = 1;
        for(int i = n; i > x; i--) {
            ans*=i;
        }
        for(int i = 1; i <= x; i++) {
            ans/=i;
        }
        return ans;
    }

    public double getLength() {
        return 0;
    }
}
