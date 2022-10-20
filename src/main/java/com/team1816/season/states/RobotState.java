package com.team1816.season.states;

import com.google.inject.Singleton;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

/**
 *  This class is responsible for logging the robot's actual states and estimated states.
 *  Including superstructure and subsystem states.
 */

@Singleton
public class RobotState {

    /** Odometry and field characterization */
    public final Field2d field = new Field2d();
    public Pose2d fieldToVehicle = Constants.EmptyPose;
    public Pose2d extrapolatedFieldToVehicle = Constants.EmptyPose;
    public Rotation2d vehicleToTurret = Constants.EmptyRotation;
    public ChassisSpeeds deltaVehicle = new ChassisSpeeds();
    public ChassisSpeeds normalizedDeltaChassisSpeeds = new ChassisSpeeds();
    public Double[] triAxialAcceleration = new Double[] { 0d, 0d, 0d };
    public boolean isPoseUpdated = true;

    /** Orchestrator states */
    public Orchestrator.STATE superstructureState = Orchestrator.STATE.LITTLE_MAN;
    public List<Point> visibleTargets = new ArrayList<>();
    public Collector.STATE collectorState = Collector.STATE.STOP;
    public Shooter.STATE shooterState = Shooter.STATE.STOP;
    public Spindexer.STATE spinState = Spindexer.STATE.STOP;
    public Elevator.STATE elevatorState = Elevator.STATE.STOP;
    public Cooler.STATE coolState = Cooler.STATE.WAIT;
    public double drivetrainTemp = 0;

    public RobotState() {
        SmartDashboard.putData("Field", field);
        resetPosition();
    }

    /** Resetting state */
    public synchronized void resetPosition(
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        resetPosition(initial_field_to_vehicle);
        vehicleToTurret = initial_vehicle_to_turret;
    }

    public synchronized void resetPosition(Pose2d initial_field_to_vehicle) {
        fieldToVehicle = initial_field_to_vehicle;
    }

    public synchronized void resetPosition() {
        resetPosition(Constants.kDefaultZeroingPose);
    }

    public synchronized void resetAllStates() {
        superstructureState = Orchestrator.STATE.LITTLE_MAN;
        collectorState = Collector.STATE.STOP;
        spinState = Spindexer.STATE.STOP;
        elevatorState = Elevator.STATE.STOP;
        shooterState = Shooter.STATE.STOP;
        coolState = Cooler.STATE.WAIT;
        deltaVehicle = new ChassisSpeeds();
        normalizedDeltaChassisSpeeds = new ChassisSpeeds();
        triAxialAcceleration = new Double[] { 0d, 0d, 0d };
        isPoseUpdated = true;
        visibleTargets.clear();
        drivetrainTemp = 0;
    }

    /** Base State getters */
    public synchronized Pose2d getLatestFieldToVehicle() {
        return fieldToVehicle;
    }

    public Rotation2d getLatestFieldToTurret() {
        return fieldToVehicle.getRotation().plus(vehicleToTurret);
    }

    public synchronized Pose2d getFieldToTurretPos() {
        return new Pose2d(
            fieldToVehicle
                .transformBy(
                    new Transform2d(new Translation2d(-.1, .1), Constants.EmptyRotation)
                )
                .getTranslation(),
            getLatestFieldToTurret()
        );
    }

    public synchronized Pose2d getEstimatedFieldToTurretPos() {
        return new Pose2d(
            extrapolatedFieldToVehicle
                .transformBy(
                    new Transform2d(new Translation2d(-.1, .1), Constants.EmptyRotation)
                )
                .getTranslation(),
            getLatestFieldToTurret()
        );
    }

    public synchronized ChassisSpeeds getCalculatedAccel() {
        return normalizedDeltaChassisSpeeds;
    }

    /** Distance calculation */
    public double getDistanceToGoal() {
        double estimatedDistanceToGoalMeters = fieldToVehicle
            .getTranslation()
            .getDistance(Constants.targetPos.getTranslation());
        System.out.println("estimated distance = " + estimatedDistanceToGoalMeters);
        return estimatedDistanceToGoalMeters;
    }

    public double getExtrapolatedDistanceToGoal() {
        double extrapolatedDistanceToGoalMeters = extrapolatedFieldToVehicle
            .getTranslation()
            .getDistance(Constants.targetPos.getTranslation());
        return extrapolatedDistanceToGoalMeters;
    }

    /** Shuffleboard real-time telemetry data */
    public synchronized void outputToSmartDashboard() {
        field.setRobotPose(fieldToVehicle);
        field.getObject("EstimatedRobot").setPose(extrapolatedFieldToVehicle);
        field.getObject(Turret.NAME).setPose(getFieldToTurretPos());
    }

    /** Camera state */
    public static class Point {
        public int id; // -2 if not detected

        public double x;
        public double y;
        public double z;

        public double weight;

        public Point() {
            id = 0;
            x = 0;
            y = 0;
            z = 0;
            weight = 0;
        }
    }
}
