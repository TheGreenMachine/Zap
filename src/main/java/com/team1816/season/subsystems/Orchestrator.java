package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

@Singleton
public class Orchestrator {

    // this class deals with organizing subsystems into actions

    @Inject
    private static RobotState robotState;

    @Inject
    private static Collector collector;

    @Inject
    private static Spindexer spindexer;

    @Inject
    private static Elevator elevator;

    @Inject
    private static Shooter shooter;

    // for automatic value setting
    @Inject
    private static DistanceManager distanceManager;

    @Inject
    private static Camera camera;
    
    private boolean collecting;
    private boolean revving;
    private boolean firing;
    private final boolean useVision;

    public Orchestrator() {
        collecting = false;
        revving = false;
        firing = false;
        useVision = Constants.kUseVision;
    }

    public void setStopped(boolean notCoasting) {
        collector.setState(Collector.COLLECTOR_STATE.STOP); // stop states auto-set subsystems to stop moving
        spindexer.setState(Spindexer.SPIN_STATE.STOP);
        elevator.setState(Elevator.ELEVATOR_STATE.STOP);
        if(notCoasting){
            shooter.setState(Shooter.SHOOTER_STATE.STOP);
        } else {
            shooter.setState(Shooter.SHOOTER_STATE.COASTING);
        }
        collecting = false;
        revving = false;
        firing = false;
        System.out.println("stopping/starting orchestrator");
    }

    public void setCollecting(boolean backSpin) {
        setCollecting(!collecting, backSpin);
    }

    public void setCollecting(boolean collecting, boolean backSpin) {
        this.collecting = collecting;
        if(collecting){
            if(backSpin){
                collector.setState(Collector.COLLECTOR_STATE.FLUSH);
            } else {
                collector.setState(Collector.COLLECTOR_STATE.COLLECTING);
            }
            if(!firing){ // TODO set up logic to minimize / remove all ifs
                spindexer.setState(Spindexer.SPIN_STATE.COLLECT);
            }
        } else {
            collector.setState(Collector.COLLECTOR_STATE.STOP);
            if(!firing){
                spindexer.setState(Spindexer.SPIN_STATE.STOP);
            }
        }
    }

    public void setRevving(boolean revving, double shooterVel) {
        this.revving = revving;
        camera.setEnabled(useVision);
        if(revving){
            System.out.println("revving!");
            shooter.setState(Shooter.SHOOTER_STATE.REVVING);
            if (useVision) {
                shooter.setVelocity(getDistance(DistanceManager.SUBSYSTEM.SHOOTER));
                shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) == 1);
            } else {
                shooter.setVelocity(shooterVel);
            }
        } else {
            shooter.setState(Shooter.SHOOTER_STATE.COASTING);
        }
    }

    public void setFiring(boolean firing) {
        this.firing = firing;
        if(firing){
            if (!elevator.colorOfBall()) { // spit out ball if wrong color
                shooter.setHood(false);
            } else if (shooter.isVelocityNearTarget()) { // only fire if - NEEDS DEBUGGING !!!!! - later introduce robotState.isStationary()
                if (useVision) {
                    spindexer.setSpindexer(getDistance(DistanceManager.SUBSYSTEM.SPINDEXER)); // is this needed in buckets? - TODO
                    elevator.autoElevator(getDistance(DistanceManager.SUBSYSTEM.ELEVATOR));
                    shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) > 0);
                }
            } else {
                return; // do not switch states to firing if not close to desired velocity && is our color of ball
            }
            System.out.println("FIRING!");
            spindexer.setState(Spindexer.SPIN_STATE.FIRE);
            elevator.setState(Elevator.ELEVATOR_STATE.FIRE);
        } else {
            if(!collecting){
                spindexer.setState(Spindexer.SPIN_STATE.STOP);
            }
            elevator.setState(Elevator.ELEVATOR_STATE.STOP);
        }
    }

    public double getDistance(DistanceManager.SUBSYSTEM subsystem) {
        System.out.println("GETTING DISTANCE FROM CAMERA / DISTANCE MANAGER " + distanceManager.getOutput(camera.getDistance(), subsystem));
        return distanceManager.getOutput(camera.getDistance(), subsystem);
    }


    public double getPredictedDistance() {
        Translation2d shooterDist = new Translation2d(
            distanceManager.getOutput(camera.getDistance(), DistanceManager.SUBSYSTEM.SHOOTER),
            Rotation2d.fromDegrees(robotState.getLatestFieldToTurret()));
        Translation2d motionBuffer = new Translation2d(robotState.delta_field_to_vehicle.dx, robotState.delta_field_to_vehicle.dy);
        return (motionBuffer.plus(shooterDist)).getNorm();
    }
}
