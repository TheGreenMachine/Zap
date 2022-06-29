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
                var name = module.getModuleName();
                var prefix = "Drivetrain/" + name;
                // Azimuth
                subsystem.CreateBadLogTopic(
                    prefix + "AzimuthPosition",
                    "ticks",
                    module::getActualAzimuth,
                    "hide",
                    "join:Drivetrain/AzimuthPosition"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "AzimuthDemand",
                    "ticks",
                    module::getDesiredAzimuth,
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
                    module::getActualDrive,
                    "hide",
                    "join:Drivetrain/DriveVelocity"
                );
                subsystem.CreateBadLogTopic(
                    prefix + "DriveVelocityDemand",
                    "ticks",
                    module::getDesiredDrive,
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
                subsystem.CreateBadLogTopic(
                    prefix + "DriveTemperature",
                    "degrees C",
                    module::getMotorTemp,
                    "hide",
                    "join:Drivetrain/Temperature"
                );
            }
        } else {
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityTicksActual,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityTicksActual,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityTicksDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityTicksDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/LeftError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftError,
                "hide",
                "join:Drivetrain/VelocityError"
            );
            subsystem.CreateBadLogTopic(
                "Drivetrain/RightError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightError,
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
            drivetrain::getActualHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
        subsystem.CreateBadLogTopic(
            "Drivetrain/DesiredHeading",
            "Angle",
            drivetrain::getDesiredHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
    }
}
