package com.team1816.season.states;

import static com.team1816.lib.subsystems.Subsystem.factory;
import static com.team1816.lib.subsystems.Subsystem.robotState;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;

/** The class responsible for organizing the collector, spindexer, elevator, and shooter into runnable actions - manages the robot's DESIRED states */

@Singleton
public class Orchestrator {

    /** subsystems */
    private static Drive drive;
    private static Turret turret;
    private static Collector collector;
    private static Spindexer spindexer;
    private static Elevator elevator;
    private static Shooter shooter;
    private static LedManager ledManager;

    private static DistanceManager distanceManager;
    private static Camera camera;

    /** state */
    private STATE superstructureState;
    private boolean collecting;
    private boolean revving;
    private boolean firing;
    private final boolean useVision;
    private final double maxAllowablePoseError = factory.getConstant(
        "maxAllowablePoseError",
        0.2
    );

    @Inject
    public Orchestrator(
        DistanceManager dm,
        Camera cam,
        Drive.Factory df,
        Turret tur,
        Collector col,
        Spindexer spin,
        Elevator elev,
        Shooter shoot,
        LedManager led
    ) {
        drive = df.getInstance();
        distanceManager = dm;
        camera = cam;
        turret = tur;
        collector = col;
        spindexer = spin;
        elevator = elev;
        shooter = shoot;
        ledManager = led;
        superstructureState = STATE.FAT_BOY;
        collecting = false;
        revving = false;
        firing = false;
        useVision = Constants.kUseVision;
    }

    /** actions */
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
            if (turret.getControlMode() == Turret.ControlMode.ABSOLUTE_FOLLOWING) {
                setSuperstructureState(STATE.FAT_BOY);
            } else {
                setSuperstructureState(STATE.LITTLE_MAN);
            }
            if (superstructureState == STATE.FAT_BOY) {
                fatBoy();
            } else {
                if (camera.isEnabled() && !manual) {
                    superstructureState = STATE.LITTLE_MAN;
                    littleMan();
                } else {
                    shooter.setVelocity(shooterVel);
                }
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

    public void autoAim() {
        if (Constants.kUseVision) {
            ledManager.setCameraLed(true);
            camera.setCameraEnabled(true);
            turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
            ledManager.setDefaultStatus(LedManager.RobotStatus.SEEN_TARGET);
        } else {
            System.out.println("can't auto aim b/c camera not on");
        }
    }

    /** superstructure state */
    public void setSuperstructureState(STATE state) {
        superstructureState = state;
        robotState.superstructureState = superstructureState;
    }

    public void fatBoy() {
        if (
            turret.getControlMode() != Turret.ControlMode.EJECT &&
            turret.getControlMode() != Turret.ControlMode.ABSOLUTE_FOLLOWING
        ) {
            turret.setControlMode(Turret.ControlMode.ABSOLUTE_FOLLOWING);
        }
        double distance = robotState.getExtrapolatedDistanceToGoal();
        shooter.setVelocity(
            distanceManager.getOutput(distance, DistanceManager.SUBSYSTEM.SHOOTER)
        );
    }

    public void littleMan() {
        shooter.setVelocity(getOutput(DistanceManager.SUBSYSTEM.SHOOTER));
    }

    /** update subsystem state */
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

    public double getOutput(DistanceManager.SUBSYSTEM subsystem) {
        if (useVision) {
            double dist = robotState.getDistanceToGoal();
            System.out.println("tracked distance to goal is . . . " + dist);
            return distanceManager.getOutput(dist, subsystem);
        } else {
            System.out.println("using neither poseTracking nor vision ! - not intended");
            return -1;
        }
    }

    public boolean needsVisionUpdate() { // true means that the pose needs to be updated
        if (!robotState.isPoseUpdated) {
            return true;
        }
        if (RobotBase.isSimulation() || RobotBase.isReal()) return false; // simulation requires too much effort, reality can't happen right now
        boolean needsVisionUpdate =
            (
                Math.abs(
                    robotState.getCalculatedAccel().vxMetersPerSecond -
                    robotState.triAxialAcceleration[0]
                ) >
                Constants.kMaxAccelDiffThreshold ||
                Math.abs(
                    robotState.getCalculatedAccel().vyMetersPerSecond -
                    robotState.triAxialAcceleration[1]
                ) >
                Constants.kMaxAccelDiffThreshold ||
                Math.abs(-9.8d - robotState.triAxialAcceleration[2]) >
                Constants.kMaxAccelDiffThreshold
            );
        if (needsVisionUpdate) {
            robotState.isPoseUpdated = false;
        }
        return needsVisionUpdate; // placeHolder
    }

    public Pose2d calculatePoseFromCamera() {
        var cameraPoints = robotState.visibleTargets;
        List<Pose2d> poses = new ArrayList<>();
        double sX = 0, sY = 0;
        for (RobotState.Point point : cameraPoints) {
            Pose2d targetPos = new Pose2d(
                Constants.fieldTargets.get(point.id).getX(),
                Constants.fieldTargets.get(point.id).getY(),
                new Rotation2d()
            );
            if (point.id == -1) { // adding hub radius target offset - this is for retro-reflective tape only
                double x, y;
                x =
                    Units.inchesToMeters(Constants.kTargetRadius) *
                    point.x /
                    (Math.sqrt(point.x * point.x + point.y * point.y));
                y =
                    Units.inchesToMeters(Constants.kTargetRadius) *
                    point.y /
                    (Math.sqrt(point.x * point.x + point.y * point.y));
                point.x += x;
                point.y += y;
            }
            Pose2d p = targetPos.plus(
                new Transform2d(
                    new Translation2d(point.x, point.y),
                    robotState
                        .getLatestFieldToTurret()
                        .rotateBy(Rotation2d.fromDegrees(180))
                )
            ); // inverse turret angle
            sX += p.getX();
            sY += p.getY();
            poses.add(p);
        }
        if (cameraPoints.size() > 0) {
            Pose2d pose = new Pose2d(
                sX / cameraPoints.size(),
                sY / cameraPoints.size(),
                robotState.fieldToVehicle.getRotation()
            );
            robotState.isPoseUpdated = true;
            return pose;
        }
        return robotState.fieldToVehicle;
    }

    public void updatePoseWithCamera() {
        Pose2d newRobotPose = calculatePoseFromCamera();
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
            robotState.isPoseUpdated = true;
        }
    }

    public enum STATE {
        FAT_BOY,
        LITTLE_MAN,
    }
}