package com.team254.lib.control;

import com.team1816.season.Robot;
import com.team1816.lib.hardware.RobotFactory;
import com.team254.lib.util.SynchronousPIDF;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Singleton;

@Singleton
public class SwerveHeadingController {
    private static SwerveHeadingController INSTANCE;

    // State
    private double mError;
    private double targetHeading;
    private double disabledTimestamp;
    private double lastUpdateTimestamp;
    private final SynchronousPIDF stabilizationPID;
    private final SynchronousPIDF snapPID;
    private final SynchronousPIDF stationaryPID;

    public enum State{
        Off, Stabilize, Snap, TemporaryDisable, Stationary
    }

    private State currentState = State.Off;

    // Constants
    public static final RobotFactory factory = Robot.getFactory();
    public static final double SNAP_kP = factory.getConstant("heading_kP");
    public static final double SNAP_kI = factory.getConstant("heading_kI");
    public static final double SNAP_kD = factory.getConstant("heading_kD");
    public static final double SNAP_kF = factory.getConstant("heading_kF");
    private static final double DISABLE_TIME_LENGTH = 0.2;

    public SwerveHeadingController(){
        if (true){
            stabilizationPID = new SynchronousPIDF(0.005, 0.0, 0.0005, 0.0);
            snapPID = new SynchronousPIDF(SNAP_kP, SNAP_kI, SNAP_kD, SNAP_kF);
            stationaryPID = new SynchronousPIDF(0.01, 0.0, 0.002, 0.0);
        }else{
            stabilizationPID = new SynchronousPIDF(0.005, 0.0, 0.0005, 0.0);
            snapPID = new SynchronousPIDF(0.015, 0.0, 0.0, 0.0);
            stationaryPID = new SynchronousPIDF(0.01, 0.0, 0.002, 0.0);
        }

        targetHeading = 0;
        lastUpdateTimestamp = Timer.getFPGATimestamp();
    }

    public void setStabilizationTarget(double angle){
        targetHeading = angle;
        setState(State.Stabilize);
    }

    public void setSnapTarget(double angle){
        targetHeading = angle;
        setState(State.Snap);
    }

    public void setStationaryTarget(double angle){
        targetHeading = angle;
        setState(State.Stationary);
    }

    public State getState(){
        return currentState;
    }

    private void setState(State newState){
        currentState = newState;
    }

    public void disable(){
        setState(State.Off);
    }

    public void temporarilyDisable(){
        setState(State.TemporaryDisable);
        disabledTimestamp = Timer.getFPGATimestamp();
    }

    public double getTargetHeading(){
        return targetHeading;
    }

    public double getError() {
        return mError;
    }

    public double updateRotationCorrection(double heading, double timestamp){
        double correction = 0;
        double error = heading - targetHeading;
        mError = error;
        double dt = timestamp - lastUpdateTimestamp;

        switch(currentState){
            case Off:

                break;
            case TemporaryDisable:
                targetHeading = heading;
                if(timestamp - disabledTimestamp >= DISABLE_TIME_LENGTH)
                    setState(State.Stabilize);
                break;
            case Stabilize:
                correction = stabilizationPID.calculate(error, dt);
                break;
            case Snap:
                correction = snapPID.calculate(error, dt);
                break;
            case Stationary:
                correction = stationaryPID.calculate(error, dt);
                break;
        }

        lastUpdateTimestamp = timestamp;
        return correction;
    }

}
