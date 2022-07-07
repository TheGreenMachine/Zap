package com.team1816.season.states;

import static com.team1816.lib.subsystems.Subsystem.factory;
import static com.team1816.lib.subsystems.Subsystem.robotState;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.math.PoseUtil;
import com.team1816.lib.subsystems.Drive;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/* class responsible for organizing the collector, spindexer, elevator, and shooter into runnable actions - manages the robot's DESIRED states */

@Singleton
public class Superstructure {

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
    private double maxAllowablePoseError = factory.getConstant(
        "maxAllowablePoseError",
        0.2
    );

    public Superstructure() {
        drive = driveFactory.getInstance();
        collecting = false;
        revving = false;
        firing = false;
        useVision = Constants.kUseVision;
    }

    public void setStopped(boolean notCoasting) {
        collector.setDesiredState(Collector.STATE.STOP);
        elevator.setDesiredState(Elevator.STATE.STOP);
        if (notCoasting) {
            shooter.setDesiredState(Shooter.STATE.STOP);
            spindexer.setDesiredState(Spindexer.STATE.STOP);
        } else {
            shooter.setDesiredState(Shooter.STATE.COASTING);
            spindexer.setDesiredState(Spindexer.STATE.COAST);
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
        updateDesiredSpindexer(backSpin);
        updateDesiredElevator();
        updateDesiredCollector(backSpin);
    }

    public void setRevving(boolean revving, double shooterVel) {
        setRevving(revving, shooterVel, false);
    }

    public void setRevving(boolean revving, double shooterVel, boolean manual) {
        this.revving = revving;
        System.out.println("struct - rev " + revving);
        if (revving) {
            shooter.setDesiredState(Shooter.STATE.REVVING);
            if (camera.isEnabled() && !manual) {
                shooter.setVelocity(getDistance(DistanceManager.SUBSYSTEM.SHOOTER));
            } else {
                shooter.setVelocity(shooterVel);
            }
        } else {
            shooter.setDesiredState(Shooter.STATE.COASTING);
        }
        updateDesiredSpindexer(false);
        updateDesiredCollector(false);
    }

    public void setFiring(boolean firing) {
        this.firing = firing;
        System.out.println("struct - fire " + firing);
        updateDesiredSpindexer(false);
        updateDesiredElevator();
        updateDesiredCollector(false);
    }

    public void updateDesiredCollector(boolean backspin) {
        if (collecting) {
            if (backspin) {
                collector.setDesiredState(Collector.STATE.FLUSH);
            } else {
                collector.setDesiredState(Collector.STATE.COLLECTING);
            }
        } else if (revving) {
            collector.setDesiredState(Collector.STATE.REVVING);
        } else {
            collector.setDesiredState(Collector.STATE.STOP);
        }
    }

    public void updateDesiredSpindexer(boolean backSpin) {
        if (firing) {
            spindexer.setDesiredState(Spindexer.STATE.FIRE);
        } else if (collecting) {
            if (backSpin) {
                spindexer.setDesiredState(Spindexer.STATE.FLUSH);
            } else {
                spindexer.setDesiredState(Spindexer.STATE.COLLECT);
            }
        } else if (revving) {
            spindexer.setDesiredState(Spindexer.STATE.INDEX);
        } else {
            spindexer.setDesiredState(Spindexer.STATE.COAST);
        }
    }

    public void updateDesiredElevator() {
        if (firing) {
            elevator.setDesiredState(Elevator.STATE.FIRE);
        } else {
            elevator.setDesiredState(Elevator.STATE.INTAKE);
        }
    }

    public double getDistance(DistanceManager.SUBSYSTEM subsystem) {
        if (useVision) {
            double camDis = camera.getDistance();
            System.out.println("tracked camera distance is . . . " + camDis);
            return distanceManager.getOutput(camDis, subsystem);
        } else {
            System.out.println("using neither poseTracking nor vision ! - not intended");
            return -1;
        }
    }

    public void shootWhileMoving(double currentCamDist) {
        Translation2d chassisVelocity = new Translation2d(
            robotState.deltaVehicle.vxMetersPerSecond,
            robotState.deltaVehicle.vyMetersPerSecond
        );
        Translation2d shooterDirection = new Translation2d( //important to make sure that this is a unit vector
            1,
            robotState.getLatestFieldToTurret()
        );

        double extrapolatedShooterOutput = distanceManager.getOutput(
            shooter.convertShooterMetersToTicksPerSecond(
                getBallVel(currentCamDist) - //get velocity of ball
                chassisVelocity.getNorm() *
                Math.cos(PoseUtil.getAngleBetween(chassisVelocity, shooterDirection))
            ),
            DistanceManager.SUBSYSTEM.SHOOTER
        );

        // setting velocity
        shooter.setVelocity(extrapolatedShooterOutput);
    }

    public double getBallVel(double distance) {
        return 0.0248 * distance - 0.53;
    }

    public void updatePoseWithCamera() {
        double cameraDist = camera.getDistance();
        // 26.56 = radius of center hub - - 5629 = square of height of hub
        double distanceToCenterMeters = Units.inchesToMeters(
            26.56 + (Math.sqrt((cameraDist * cameraDist) - 5629.5))
        );

        Translation2d deltaToHub = new Translation2d(
            distanceToCenterMeters,
            robotState.getLatestFieldToTurret()
        );
        Pose2d newRobotPose = Constants.targetPos.transformBy(
            new Transform2d(
                deltaToHub.unaryMinus(),
                robotState.fieldToVehicle.getRotation()
            )
        );
        if (
            Math.abs(
                Math.hypot(
                    robotState.fieldToVehicle.getX() - newRobotPose.getX(),
                    robotState.fieldToVehicle.getY() - newRobotPose.getY()
                )
            ) >
            maxAllowablePoseError
        ) {
            System.out.println(newRobotPose + " = new robot pose");
            drive.resetOdometry(newRobotPose);
            robotState.fieldToVehicle = newRobotPose;
        }
    }
}
