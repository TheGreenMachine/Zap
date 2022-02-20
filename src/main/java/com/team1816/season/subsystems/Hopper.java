package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.sendable.SendableBuilder;

@Singleton
public class Hopper extends Subsystem {
    // from what I understand, we want to make the elevator use velocity control
    // and also be in tandem with the shooter (w/ distance manager) so that depending on the shot distance,
    // the elevator will be given a wee bit more or less umph

    // this class will now deal with organizing the collector, spindexer, elevator, and shooter -- heck this might as well just be a SUPERSTRUCTURE

    private static final String NAME = "hopper";

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


    // State
    private boolean outputsChanged;

    private boolean stopped = false;
    private boolean collecting = false;
    private boolean flushing = false;
    private boolean revving = false;
    private boolean firing = false;
    private final boolean isAutoAim = factory.getConstant("useAutoAim") < 0;

    //band-aid patches
    private double COAST_VELOCIY = 9_000; // tune this and make changeable with a button in shooter itself


    public Hopper() {
        super(NAME);
    }

    public void setStopped(boolean stopped){
        this.stopped = stopped;
        if(stopped){
            collecting = false;
            flushing = false;
            revving = false;
            firing = false;
        }
    }

    public void setStopped(){
        setStopped(!stopped);
        System.out.println("starting/stopping hopper");
    }

    public void setFlushing(boolean flushing){
        this.flushing = flushing;
        if(flushing){
            stopped = false;
            collecting = false;
            revving = false;
            firing = false;
        }
    }

    public void setCollecting(boolean collecting){
        this.collecting = collecting;
    }

    public void setRevving(boolean revving){
        this.revving = revving;
    }

    public void setFiring(boolean firing){
        this.firing = firing;
    }

    public void stopAll(){
        shooter.setVelocity(0); // TODO make shooter use states as well
        collector.setState(Collector.COLLECTOR_STATE.STOP); // stop states auto-set subsystems to stop moving
        spindexer.setState(Spindexer.SPIN_STATE.STOP);
        elevator.setState(Elevator.ELEVATOR_STATE.STOP);
        shooter.setState(Shooter.SHOOTER_STATE.STOP);
    }

    public void flush(){
        elevator.autoElevator(-1);
        shooter.setVelocity(COAST_VELOCIY);
        collector.setState(Collector.COLLECTOR_STATE.FLUSH);
        spindexer.setState(Spindexer.SPIN_STATE.FLUSH);
        elevator.setState(Elevator.ELEVATOR_STATE.FLUSH);
        shooter.setState(Shooter.SHOOTER_STATE.COASTING);
    }

    public void collect(){
        collector.setState(Collector.COLLECTOR_STATE.COLLECTING);
        if(!firing){
            spindexer.setState(Spindexer.SPIN_STATE.INTAKE);
        }
    }

    public void revUp(){
        shooter.setState(Shooter.SHOOTER_STATE.REVVING);
        if(isAutoAim){
            shooter.setVelocity(getDistance(DistanceManager.SUBSYSTEM.SHOOTER));
            shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) == 1);
        } else {
            shooter.setVelocity(Shooter.MID_VELOCITY);
        }
        if(!collecting){
            collector.setState(Collector.COLLECTOR_STATE.REVVING);
            if(!collecting){
                spindexer.setState(Spindexer.SPIN_STATE.STOP);
            }
        }
        if(!firing){ // make the elevator flush unless firing
            elevator.setState(Elevator.ELEVATOR_STATE.FLUSH);
            elevator.autoElevator(-0.25);
        }
    }

    public void fire(){
        if(!elevator.colorOfBall()){ // spit out ball if wrong color ? idk maybe make this into a flush command
            shooter.setHood(false);
        } else if(shooter.isVelocityNearTarget()){ // only fire if
            if(isAutoAim){
                spindexer.autoSpindexer(getDistance(DistanceManager.SUBSYSTEM.SPINDEXER));
                elevator.autoElevator(getDistance(DistanceManager.SUBSYSTEM.ELEVATOR));
                shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) > 0);
            }
        } else {
            return; // do not switch states to firing if not close to desired velocity
        }

        elevator.setState(Elevator.ELEVATOR_STATE.FIRING);
        spindexer.setState(Spindexer.SPIN_STATE.INTAKE);
    }

    public double getDistance(DistanceManager.SUBSYSTEM subsystem){
        return distanceManager.getOutput(camera.getDistance(), subsystem);
    }

    public void idle(){ // not rly efficient organization rn but easier to understand - each action has its own priority over idling
        if(!collecting && !revving){
            collector.setState(Collector.COLLECTOR_STATE.STOP);
        }

        if(!firing && ! collecting && !revving){
            spindexer.setState(Spindexer.SPIN_STATE.STOP);
        }

        if(!firing && !revving){
            elevator.setState(Elevator.ELEVATOR_STATE.STOP);
        }

        if(!firing){
            shooter.setState(Shooter.SHOOTER_STATE.COASTING);
            shooter.setVelocity(COAST_VELOCIY);
        }
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) { // boolean land!
            if(stopped) {
                stopAll();
            } else if(flushing){
                flush();
            } else {
                if(firing){
                    fire();
                }
                if(revving){
                    revUp();
                }
                if(collecting){
                    collect();
                }
                idle(); // idle all subsystems not in use
            }
            outputsChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }
}
