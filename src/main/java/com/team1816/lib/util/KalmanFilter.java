package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {

    public Pose2d estimate = new Pose2d();
    public double kalman_gain[][] = new double[][] {{1, 0}, {0, 1}};
    public double covariance[][] = new double[][] {{1, 0}, {0, 1}};
    public double estimation_uncertainty[][] = new double[][] {{0, 0}, {0, 0}};
    public double time_step_delta = 0.002;


    public KalmanFilter(Pose2d initial_estimate, double[][] kalman_gain, double[][] covariance, double time_step_delta) {
        this.estimate = initial_estimate;
        this.kalman_gain = kalman_gain;
        this.covariance = covariance
        this.time_step_delta = time_step_delta;
    }

    public void update(Pose2d estimate, Pose2d measurement, double[][] measurement_uncertainty, double[][] process_noise_uncertainty, double[][] estimation_uncertainty) {
        this.estimate = estimate;
        SimpleMatrix F = new SimpleMatrix(new double[][] {{1d, time_step_delta}, {0, 1d}});
        SimpleMatrix P = new SimpleMatrix(estimation_uncertainty);
        SimpleMatrix Q = new SimpleMatrix(process_noise_uncertainty);
        this.estimation_uncertainty = matrixToArray(F.mult(P).mult(F.transpose()).plus(Q));

    }

    public double[][] matrixToArray(SimpleMatrix matrix) {
        double[][] array = new double[matrix.numRows()][matrix.numCols()];
        for (int r = 0; r < matrix.numRows(); r++) {
            for (int c = 0; c < matrix.numCols(); c++) {
                array[r][c] = matrix.get(r, c);
            }
        }
        return array;
    }

}
