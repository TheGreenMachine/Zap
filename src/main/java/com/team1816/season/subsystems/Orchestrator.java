package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

import static com.team1816.lib.subsystems.Subsystem.factory;

@Singleton
public class Orchestrator {

    // from what I understand, we want to make the elevator use velocity control
    // and also be in tandem with the shooter (w/ distance manager) so that depending on the shot distance,
    // the elevator will be given a wee bit more or less umph

    // this class will now deal with organizing the collector, spindexer, elevator, and shooter -- heck this might as well just be a SUPERSTRUCTURE

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
    
    private boolean stopped = false;
    private boolean collecting = false;
    private boolean revving = false;
    private boolean firing = false;
    private final boolean isAutoAim = factory.getConstant("useAutoAim") > 0;

    public Orchestrator() {}

    public void setStopped() {
        collector.setState(Collector.COLLECTOR_STATE.STOP); // stop states auto-set subsystems to stop moving
        spindexer.setState(Spindexer.SPIN_STATE.STOP);
        elevator.setState(Elevator.ELEVATOR_STATE.STOP);
        shooter.setState(Shooter.SHOOTER_STATE.STOP);
        stopped = true;
        collecting = false;
        revving = false;
        firing = false;
        System.out.println("stopping orchestrator");
    }

    public void setCollecting() {
        setCollecting(!collecting);
    }

    public void setCollecting(boolean collecting) {
        this.collecting = collecting;
        if(collecting){
            collector.setState(Collector.COLLECTOR_STATE.COLLECTING);
            if(!firing){
                spindexer.setState(Spindexer.SPIN_STATE.COLLECT);
            }
        } else {
            collector.setState(Collector.COLLECTOR_STATE.STOP);
            spindexer.setState(Spindexer.SPIN_STATE.STOP);
        }
    }

    public void setRevving(boolean revving, double shooterVel) {
        this.revving = revving;
        if(revving){
            System.out.println("revving!");
            shooter.setState(Shooter.SHOOTER_STATE.REVVING);
            if (isAutoAim) {
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
            } else if (shooter.isVelocityNearTarget()) { // only fire if - NEEDS DEBUGGING !!!!!
                if (isAutoAim) {
                    spindexer.setSpindexer(getDistance(DistanceManager.SUBSYSTEM.SPINDEXER));
                    elevator.autoElevator(getDistance(DistanceManager.SUBSYSTEM.ELEVATOR));
                    shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) > 0);
                }
            } else {
                return; // do not switch states to firing if not close to desired velocity && is our color of ball
            }
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
        System.out.println("CAMERA DISTANCE Line 163 " + distanceManager.getOutput(camera.getDistance(), subsystem));
        return distanceManager.getOutput(camera.getDistance(), subsystem);
    }
}
