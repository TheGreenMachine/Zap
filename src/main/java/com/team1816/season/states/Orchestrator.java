package com.team1816.season.states;

import static com.team1816.lib.subsystems.Subsystem.factory;
import static com.team1816.lib.subsystems.Subsystem.robotState;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.turret.Turret;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Main superstructure-style class and logical operator for handling and delegating subsystem tasks. Consists of an integrated
 * drivetrain, turret, collector, spindexer, elevator, shooter, and ledManager and utilizes closed loop state dependent
 * control via RobotState.
 * @see RobotState
 */
@Singleton
public class Orchestrator {

    /** Subsystems */
    private static Drive drive;
    private static Turret turret;
    private static Collector collector;
    private static Spindexer spindexer;
    private static Elevator elevator;
    private static Shooter shooter;
    private static LedManager ledManager;

    private static DistanceManager distanceManager;
    private static Camera camera;

    /** State */
    private STATE superstructureState;
    private boolean collecting;
    private boolean revving;
    private boolean firing;
    private final boolean useVision;
    private final double maxAllowablePoseError = factory.getConstant(
        "maxAllowablePoseError",
        4
    );
    private final double minAllowablePoseError = factory.getConstant(
        "minAllowablePoseError",
        0.15
    );

    /**
     * Instantiates an Orchestrator with all its subsystems
     * @param dm DistanceManager
     * @param cam Camera
     * @param df Drive.Factory (derives drivetrain)
     * @param tur Turret
     * @param col Collector
     * @param spin Spindexer
     * @param elev Elevator
     * @param shoot Shooter
     * @param led LedManager
     */
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

    /** Actions */

    /**
     * Stops all actions on the robot
     * @param notCoasting boolean shooterCoasting
     */
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

    /**
     * Sets the robot to collect a ball
     * @param backSpin boolean
     */
    public void setCollecting(boolean backSpin) {
        setCollecting(!collecting, backSpin);
    }

    /**
     * Controls the collector
     * @param collecting boolean
     * @param backSpin boolean
     */
    public void setCollecting(boolean collecting, boolean backSpin) {
        this.collecting = collecting;
        updateDesiredSpindexer(backSpin);
        updateDesiredElevator();
        updateDesiredCollector(backSpin);
    }

    /**
     * Sets the robot to the revving state which is preparing to shoot
     * @param revving boolean
     * @param shooterVel desiredShooterVelocity
     */
    public void setRevving(boolean revving, double shooterVel) {
        setRevving(revving, shooterVel, false);
    }

    /**
     * Sets the robot to the revving state which is preparing to shoot based on various parameters and subsystem states
     * @param revving boolean
     * @param shooterVel desiredShooterVelocity
     * @param manual boolean isManualControl
     */
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

    /**
     * Sets the robot to fire by running the elevator
     * @param firing boolean
     */
    public void setFiring(boolean firing) {
        this.firing = firing;
        System.out.println("struct - fire " + firing);
        updateDesiredSpindexer(false);
        updateDesiredElevator();
        updateDesiredCollector(false);
    }

    /**
     * Auto-aims to a target
     */
    public void autoAim() {
        if (Constants.kUseVision) {
            ledManager.setCameraLed(true);
            camera.setCameraEnabled(true);
            turret.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
            ledManager.setDefaultStatus(LedManager.RobotStatus.SEEN_TARGET);
        } else {
            turret.setControlMode(Turret.ControlMode.TARGET_FOLLOWING);
        }
    }

    /** Superstructure State */

    /**
     * Sets the superstructure state
     * @param state STATE
     */
    public void setSuperstructureState(STATE state) {
        superstructureState = state;
        robotState.superstructureState = superstructureState;
    }

    /**
     * Logic handling for motion independent shooting and ejection of game elements
     */
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

    /**
     * Logic handling for normal shooting
     */
    public void littleMan() {
        shooter.setVelocity(getOutput(DistanceManager.SUBSYSTEM.SHOOTER));
    }

    /** Update Subsystem States */

    /**
     * Updates the desired collector state
     * @param backspin boolean
     */
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

    /**
     * Updates the desired spindexer state
     * @param backSpin boolean
     */
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

    /**
     * Updates the desired elevator state
     */
    public void updateDesiredElevator() {
        if (firing) {
            elevator.setDesiredState(Elevator.STATE.FIRE);
        } else {
            elevator.setDesiredState(Elevator.STATE.INTAKE);
        }
    }

    /**
     * Returns the distance manager controlled output for a subsystem registered in the manager
     * @param subsystem DistanceManager.SUBSYSTEM
     * @return double output
     * @see DistanceManager
     */
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

    /**
     * Returns true if the pose of the drivetrain needs to be updated in a cached boolean system
     * @return boolean
     */
    public boolean needsVisionUpdate() {
        if (!robotState.isPoseUpdated) {
            return true;
        }
        if (RobotBase.isSimulation() || RobotBase.isReal()) return false;
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

    /**
     * Calculates the absolute pose of the drivetrain based on a single target
     * @param target VisionPoint
     * @return Pose2d
     * @see VisionPoint
     */
    public Pose2d calculateSingleTargetTranslation(VisionPoint target) {
        Pose2d targetPos = new Pose2d(
            FieldConfig.fieldTargets.get(target.id).getX(),
            FieldConfig.fieldTargets.get(target.id).getY(),
            new Rotation2d()
        );
        double X = target.getX(), Y = target.getY();
        if (target.id == -1) { // adding hub radius target offset - this is for retro-reflective tape only
            double x, y;
            x =
                Units.inchesToMeters(Constants.kTargetRadius) *
                target.getX() /
                (
                    Math.sqrt(
                        target.getX() * target.getX() + target.getY() * target.getY()
                    )
                );
            y =
                Units.inchesToMeters(Constants.kTargetRadius) *
                target.getY() /
                (
                    Math.sqrt(
                        target.getX() * target.getX() + target.getY() * target.getY()
                    )
                );
            X += x;
            Y += y;
        }
        Pose2d p = targetPos.plus(
            new Transform2d(
                new Translation2d(X, Y),
                robotState.getLatestFieldToCamera().rotateBy(Rotation2d.fromDegrees(180))
            )
        ); // inverse axis angle
        return p;
    }

    /**
     * Calculates the absolute pose of the drivetrain as a function of all visible targets
     * @return Pose2d
     */
    public Pose2d calculatePoseFromCamera() {
        var cameraPoints = robotState.visibleTargets;
        List<Pose2d> poses = new ArrayList<>();
        double sX = 0, sY = 0;
        for (VisionPoint point : cameraPoints) {
            var p = calculateSingleTargetTranslation(point);
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

    /**
     * Updates the pose of the drivetrain based on specified criteria
     */
    public void updatePoseWithCamera() {
        Pose2d newRobotPose = calculatePoseFromCamera();
        if (
            Math.abs(
                Math.hypot(
                    robotState.fieldToVehicle.getX() - newRobotPose.getX(),
                    robotState.fieldToVehicle.getY() - newRobotPose.getY()
                )
            ) <
            maxAllowablePoseError &&
            Math.abs(
                Math.hypot(
                    robotState.fieldToVehicle.getX() - newRobotPose.getX(),
                    robotState.fieldToVehicle.getY() - newRobotPose.getY()
                )
            ) >
            minAllowablePoseError
        ) {
            System.out.println(newRobotPose + " = new robot pose");
            drive.resetOdometry(newRobotPose);
            robotState.fieldToVehicle = newRobotPose;
            robotState.isPoseUpdated = true;
        }
    }

    /**
     * Base enum for Orchestrator states
     */
    public enum STATE {
        FAT_BOY,
        LITTLE_MAN,
    }
}
