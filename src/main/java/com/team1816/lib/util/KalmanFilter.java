package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class KalmanFilter { //TODO: complete
    public Pose2d estimate = new Pose2d();
    public Pose2d measurement = new Pose2d();
    public double [][] processNoise = new double[2][2]; // covariance certainty of process i.e. odometry
    public double [][] measurementNoise = new double[2][2]; // covariance certainty of measurement i.e. external sensors

    //p1 = (1-k1h1)p1
}
