package com.team1816.lib.subsystems;

public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var isSwerve = drivetrain instanceof SwerveDrivetrain;
        var subsystem = (Subsystem) drivetrain;
        if (isSwerve) {
            for (
                int i = 0;
                i < 1; //((SwerveDrivetrain) drivetrain).getSwerveModules().length;
                i++
            ) {
                var module = ((SwerveDrivetrain) drivetrain).getSwerveModules()[i];
                var name = module.getSubsystemName();
                var prefix = "Drivetrain/" + name;
                // Azimuth
                subsystem.CreateBadLogTopic(
                    prefix + "AzimuthPosition",
                    "ticks",
                    module::getAzimuthActual,
                    "hide",
                    "join:Drivetrain/AzimuthPosition"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "AzimuthDemand",
                    "ticks",
                    module::getAzimuthDemand,
                    "hide",
                    "join:Drivetrain/AzimuthPosition"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "AzimuthError",
                    "ticks",
                    module::getAzimuthError,
                    "hide",
                    "join:Drivetrain/AzimuthError"
                );

                // Drive
                subsystem.CreateBadLogTopic(
                    prefix + "DriveVelocity",
                    "ticks",
                    module::getDriveActual,
                    "hide",
                    "join:Drivetrain/DriveVelocity"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "DriveVelocityDemand",
                    "ticks",
                    module::getDriveDemand,
                    "hide",
                    "join:Drivetrain/DriveVelocity"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "DriveError",
                    "ticks",
                    module::getDriveError,
                    "hide",
                    "join:Drivetrain/DriveError"
                );
            }
        } else {
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityNativeUnits,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityNativeUnits,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityError,
                "hide",
                "join:Drivetrain/VelocityError"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityError,
                "hide",
                "join:Drivetrain/VelocityError"
            );
        }
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
            drivetrain::getFieldDesiredYDistance,
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
            "Drivetrain/DesiredHeading",
            "Angle",
            drivetrain::getDesiredHeading,
            "hide",
            "join:Drivetrain/Heading"
        );
    }
}
