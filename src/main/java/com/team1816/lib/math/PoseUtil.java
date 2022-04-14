package com.team1816.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

public class PoseUtil {

    public static double getAngleBetween(Translation2d a, Translation2d b) {
        double dot = (a.getNorm() * b.getNorm() == 0)
            ? 0
            : Math.acos(
                (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm())
            );
        double cross = crossProduct(a, b);
        if (cross > 0) {
            dot *= -1;
        }
        return dot;
    }

    private static double crossProduct(Translation2d a, Translation2d b) {
        double[] vect_A = { a.getX(), a.getY(), 0 };
        double[] vect_B = { b.getX(), b.getY(), 0 };
        return vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }
}
