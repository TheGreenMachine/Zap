package com.team1816.season;

import com.google.inject.Singleton;
import com.team1816.season.subsystems.Turret;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class RobotState {

    public final Field2d field = new Field2d();
    public Pose2d field_to_vehicle = Constants.EmptyPose;
    public Rotation2d vehicle_to_turret = Constants.EmptyRotation;
    public Twist2d delta_field_to_vehicle = new Twist2d();

    public RobotState() {
        SmartDashboard.putData("Field", field);
        reset();
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        field_to_vehicle = initial_field_to_vehicle;
        vehicle_to_turret = initial_vehicle_to_turret;
    }

    public synchronized void reset() {
        reset(Constants.StartingPose, Constants.EmptyRotation);
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        // CCW rotation increases degrees
        return field_to_vehicle;
    }

    public double getLatestFieldToTurret() {
        return field_to_vehicle.getRotation().plus(vehicle_to_turret).getDegrees();
    }

    public boolean isStationary(){
        return
            delta_field_to_vehicle.dx == 0 &&
            delta_field_to_vehicle.dy == 0 &&
            delta_field_to_vehicle.dtheta == 0;
    }

    public synchronized void outputToSmartDashboard() {
        //shuffleboard periodic updates should be here
        field.setRobotPose(field_to_vehicle);
        field
            .getObject(Turret.NAME)
            .setPose(
                new Pose2d(
                    field_to_vehicle
                        .transformBy(
                            new Transform2d(
                                new Translation2d(-.1, .1),
                                Constants.EmptyRotation
                            )
                        )
                        .getTranslation(),
                    Rotation2d.fromDegrees(getLatestFieldToTurret())
                )
            );
    }
}
