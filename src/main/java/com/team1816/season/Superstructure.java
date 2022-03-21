package com.team1816.season;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

@Singleton
public class Superstructure {

    // this class deals with organizing subsystems into actions

    @Inject
    private static RobotState mRobotState;

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

    public Superstructure() {
        collecting = false;
        revving = false;
        firing = false;
        useVision = Constants.kUseVision;
    }

    public void setStopped(boolean notCoasting) {
        collector.setDesiredState(Collector.COLLECTOR_STATE.STOP); // stop states auto-set subsystems to stop moving
        spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
        elevator.setDesiredState(Elevator.ELEVATOR_STATE.STOP);
        if (notCoasting) {
            shooter.setDesiredState(Shooter.SHOOTER_STATE.STOP);
        } else {
            shooter.setDesiredState(Shooter.SHOOTER_STATE.COASTING);
        }
        collecting = false;
        revving = false;
        firing = false;
        System.out.println("stopping/starting superstructure");
    }

    public void setCollecting(boolean backSpin) {
        setCollecting(!collecting, backSpin);
    }

    public void setCollecting(boolean collecting, boolean backSpin) {
        this.collecting = collecting;
        if (collecting) {
            if (backSpin) {
                collector.setDesiredState(Collector.COLLECTOR_STATE.FLUSH);
            } else {
                collector.setDesiredState(Collector.COLLECTOR_STATE.COLLECTING);
            }
            if (!firing) {
                spindexer.setDesiredState(Spindexer.SPIN_STATE.COLLECT);
            }
        } else {
            collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
            if (!firing) {
                spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
            }
        }
    }

    public void setRevving(boolean revving, double shooterVel) {
        this.revving = revving;
        if (revving) {
            System.out.println("revving!");
            shooter.setDesiredState(Shooter.SHOOTER_STATE.REVVING);
            if (useVision) {
                camera.setEnabled(true);
                shooter.setVelocity(getDistance(DistanceManager.SUBSYSTEM.SHOOTER));
                shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) > 0);
            } else {
                shooter.setVelocity(shooterVel);
            }
            System.out.println("superstructure set to rev");
        } else {
//            camera.setEnabled(false);
            shooter.setDesiredState(Shooter.SHOOTER_STATE.COASTING);
        }
    }

    public void setFiring(boolean firing) {
        this.firing = firing;
        if (firing) {
            spindexer.setDesiredState(Spindexer.SPIN_STATE.FIRE);
            elevator.setDesiredState(Elevator.ELEVATOR_STATE.FIRE);

            if (!elevator.colorOfBall()) { // spit out ball if wrong color
                shooter.setHood(false);
            }
            if (useVision) {
                elevator.overridePower(getDistance(DistanceManager.SUBSYSTEM.ELEVATOR));
            }
            System.out.println("superstructure set to fire");
        } else {
            if (!collecting) {
                spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
            }
            elevator.setDesiredState(Elevator.ELEVATOR_STATE.STOP);
        }
    }

    public double getDistance(DistanceManager.SUBSYSTEM subsystem) {
        System.out.println(
            "GETTING DISTANCE FROM CAMERA / DISTANCE MANAGER " +
            distanceManager.getOutput(camera.getDistance(), subsystem)
        );
        return distanceManager.getOutput(camera.getDistance(), subsystem);
    }

    public double getPredictedDistance(DistanceManager.SUBSYSTEM subsystem) {
        Translation2d shooterDist = new Translation2d(
            distanceManager.getOutput(camera.getDistance(), subsystem),
            Rotation2d.fromDegrees(mRobotState.getLatestFieldToTurret())
        );
        Translation2d motionBuffer = new Translation2d(
            mRobotState.delta_field_to_vehicle.dx,
            mRobotState.delta_field_to_vehicle.dy
        );
        return (motionBuffer.plus(shooterDist)).getNorm();
    }
}
