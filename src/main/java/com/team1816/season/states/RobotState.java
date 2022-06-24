package com.team1816.season.states;

import com.google.inject.Singleton;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* class responsible with logging the robot's ACTUAL states - robot position (predicted) and superstructure subsystem actual states */

@Singleton
public class RobotState {

    public final Field2d field = new Field2d();
    public Pose2d field_to_vehicle = Constants.EmptyPose;
    public Pose2d estimated_field_to_vehicle = Constants.EmptyPose;
    public Rotation2d vehicle_to_turret = Constants.EmptyRotation;
    public ChassisSpeeds delta_vehicle = new ChassisSpeeds();
    public double shooterMPS = 0; // needs to be remapped - default value

    // Superstructure ACTUAL states
    public Point visionPoint = new Point();
    public Collector.STATE collectorState = Collector.STATE.STOP;
    public Shooter.STATE shooterState = Shooter.STATE.STOP;
    public Spindexer.STATE spinState = Spindexer.STATE.STOP;
    public Elevator.STATE elevatorState = Elevator.STATE.STOP;
    public Cooler.STATE coolState = Cooler.STATE.WAIT;

    public boolean hasOverheated = false;

    public RobotState() {
        SmartDashboard.putData("Field", field);
        resetPosition();
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void resetPosition(
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        resetPosition(initial_field_to_vehicle);
        vehicle_to_turret = initial_vehicle_to_turret;
    }

    public synchronized void resetPosition(Pose2d initial_field_to_vehicle) {
        field_to_vehicle = initial_field_to_vehicle;
    }

    public synchronized void resetPosition() {
        resetPosition(Constants.StartingPose);
    }

    public synchronized void resetAllStates(){
        collectorState = Collector.STATE.STOP;
        spinState = Spindexer.STATE.STOP;
        elevatorState = Elevator.STATE.STOP;
        shooterState = Shooter.STATE.STOP;
        coolState = Cooler.STATE.WAIT;
        delta_vehicle = new ChassisSpeeds();
        visionPoint = new Point();
        shooterMPS = 0;
        hasOverheated = false;
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        // CCW rotation increases degrees
        return field_to_vehicle;
    }

    public Rotation2d getLatestFieldToTurret() {
        return field_to_vehicle.getRotation().plus(vehicle_to_turret);
    }

    public synchronized Pose2d getFieldToTurretPos() {
        return new Pose2d(
            field_to_vehicle
                .transformBy(
                    new Transform2d(new Translation2d(-.1, .1), Constants.EmptyRotation)
                )
                .getTranslation(),
            getLatestFieldToTurret()
        );
    }

    public double getEstimatedDistanceToGoal() {
        double estimatedDistanceToGoalMeters = field_to_vehicle
            .getTranslation()
            .getDistance(Constants.targetPos.getTranslation());
        double distInches =
            (
                Math.sqrt(
                    Units.metersToInches(estimatedDistanceToGoalMeters) *
                    Units.metersToInches(estimatedDistanceToGoalMeters) +
                    (Constants.kHeightFromCamToHub * Constants.kHeightFromCamToHub)
                ) -
                Constants.kTargetRadius
            );
        System.out.println("estimated distance = " + distInches);
        return distInches;
    }

    public synchronized void outputToSmartDashboard() {
        //shuffleboard periodic updates should be here
        field.setRobotPose(field_to_vehicle);
        field.getObject(Turret.NAME).setPose(getFieldToTurretPos());
    }

    // Camera state
    public class Point {

        public double cX;
        public double cY;

        public Point() {
            cX = 0;
            cY = 0;
        }
    }
}
