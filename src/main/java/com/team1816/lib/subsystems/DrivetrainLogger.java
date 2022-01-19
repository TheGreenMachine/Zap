package com.team1816.lib.subsystems;

public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var subsystem = (Subsystem) drivetrain;
        subsystem.CreateBadLogTopic(
            "Drivetrain/LeftActVel",
            "NativeUnits",
            drivetrain::getLeftVelocityNativeUnits,
            "hide",
            "join:Drivetrain/Velocities"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/RightActVel",
            "NativeUnits",
            drivetrain::getRightVelocityNativeUnits,
            "hide",
            "join:Drivetrain/Velocities"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/LeftVel",
            "NativeUnits",
            drivetrain::getLeftVelocityDemand,
            "hide",
            "join:Drivetrain/Velocities"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/RightVel",
            "NativeUnits",
            drivetrain::getRightVelocityDemand,
            "hide",
            "join:Drivetrain/Velocities"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/LeftError",
            "NativeUnits",
            drivetrain::getLeftVelocityError,
            "hide",
            "join:Drivetrain/VelocityError"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/RightError",
            "NativeUnits",
            drivetrain::getRightVelocityError,
            "hide",
            "join:Drivetrain/VelocityError"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/X Desired",
            "Inches",
            drivetrain::getFieldDesiredXDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/Y Desired",
            "Inches",
            drivetrain::getFieldYDesiredYDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/X Actual",
            "Inches",
            drivetrain::getFieldXDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/Y Actual",
            "Inches",
            drivetrain::getFieldYDistance,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/ActualHeading",
            "Angle",
            drivetrain::getHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/Heading",
            "Angle",
            drivetrain::getDesiredHeading,
            "hide",
            "join:Drivetrain/Heading"
        );
    }
}
