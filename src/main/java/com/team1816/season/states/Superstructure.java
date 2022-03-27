package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.Drive;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/* class responsible for organizing the collector, spindexer, elevator, and shooter into runnable actions - manages the robot's DESIRED states */

@Singleton
public class Superstructure {

    @Inject
    private static RobotState robotState;

    @Inject
    private static Drive.Factory driveFactory;

    private static Drive drive;

    @Inject
    private static Turret turret;

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
    private final boolean usePoseTrack;

    public Superstructure() {
        drive = driveFactory.getInstance();
        collecting = false;
        revving = false;
        firing = false;
        useVision = Constants.kUseVision;
        usePoseTrack = Constants.kUsePoseTrack;
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
            if(!revving){
                collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
            }
            collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
            if (!firing) {
                spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
                spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
            }
        }
    }

    public void setRevving(boolean revving, double shooterVel) {
        this.revving = revving;
        System.out.println("struct - rev " + revving);
        if (revving) {
            shooter.setDesiredState(Shooter.SHOOTER_STATE.REVVING);
            if (Camera.cameraEnabled || usePoseTrack) {
                shooter.setVelocity(getDistance(DistanceManager.SUBSYSTEM.SHOOTER));
                shooter.setHood(getDistance(DistanceManager.SUBSYSTEM.HOOD) > 0);
            } else {
                shooter.setVelocity(shooterVel);
            }
            if(!collecting){
                collector.setDesiredState(Collector.COLLECTOR_STATE.REVVING);
            }
        } else {
            shooter.setDesiredState(Shooter.SHOOTER_STATE.COASTING);
            if(!collecting){
                collector.setDesiredState(Collector.COLLECTOR_STATE.STOP);
            }
        }
    }

    public void setFiring(boolean firing) {
        this.firing = firing;
        System.out.println("struct - fire " + firing);
        if (firing) {
            spindexer.setDesiredState(Spindexer.SPIN_STATE.FIRE);
            elevator.setDesiredState(Elevator.ELEVATOR_STATE.FIRE);

            if (!elevator.colorOfBall()) { // spit out ball if wrong color
                shooter.setHood(false);
            }
            if (Camera.cameraEnabled || usePoseTrack) {
                elevator.overridePower(getDistance(DistanceManager.SUBSYSTEM.ELEVATOR));
            }
        } else {
            if (!collecting) {
                spindexer.setDesiredState(Spindexer.SPIN_STATE.STOP);
            }
            elevator.setDesiredState(Elevator.ELEVATOR_STATE.STOP);
        }
    }

    public double getDistance(DistanceManager.SUBSYSTEM subsystem) {
        if(useVision){
            double camDis = camera.getDistance();
            System.out.println("tracked camera distance is . . . " + camDis);
            return distanceManager.getOutput(camDis, subsystem);
        } else if(usePoseTrack){
            System.out.println("using position to plan shooter velocity");
            return distanceManager.getOutput(calculateDistanceToGoal(), subsystem);
        } else {
            System.out.println("using neither poseTracking nor vision ! - not intended");
            return -1;
        }
    }

    public double calculateDistanceToGoal(){
        double distanceToGoalMeters = robotState.field_to_vehicle.getTranslation().getDistance(Constants.targetPos.getTranslation());
        return Units.metersToInches(distanceToGoalMeters) / 1.2;
    }

    public void updatePoseWithCamera() {
        double cameraDist = camera.getDistance();
        // 26.56 = radius of center hub - - 5629 = square of height of hub
        double distanceToCenterMeters = Units.inchesToMeters(26.56 + (Math.sqrt((cameraDist * cameraDist) + 5629.5)));

        Translation2d deltaToHub = new Translation2d(distanceToCenterMeters, Rotation2d.fromDegrees(robotState.getLatestFieldToTurret()));
        Pose2d newRobotPose = Constants.targetPos.transformBy(new Transform2d(deltaToHub.unaryMinus(), robotState.field_to_vehicle.getRotation())); //
        drive.resetOdometry(newRobotPose);
    }

    public double getPredictedDistance(DistanceManager.SUBSYSTEM subsystem) {
        Translation2d shooterDist = new Translation2d(
            distanceManager.getOutput(camera.getDistance(), subsystem),
            Rotation2d.fromDegrees(robotState.getLatestFieldToTurret())
        );
        Translation2d motionBuffer = new Translation2d(
            robotState.delta_field_to_vehicle.dx,
            robotState.delta_field_to_vehicle.dy
        );
        return (motionBuffer.plus(shooterDist)).getNorm();
    }
}
