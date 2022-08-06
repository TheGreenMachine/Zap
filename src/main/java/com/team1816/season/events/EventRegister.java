package com.team1816.season.events;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.events.EventAggregator;
import com.team1816.lib.subsystems.Drive;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;

/**
 * List of all used events
 */

@Singleton
public class EventRegister {

    public static EventAggregator eventManager;

    @Inject
    private static RobotState robotState;

    @Inject
    private static IControlBoard controlBoard;

    @Inject
    private static Superstructure superstructure;

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

    @Inject
    private static Climber climber;

    @Inject
    private static LedManager ledManager;

    @Inject
    private static DistanceManager distanceManager;

    @Inject
    private static Camera camera;

    public EventRegister() {
        eventManager = new EventAggregator();
        drive = driveFactory.getInstance();
    }

    public void subscribeToControlBoard() {

    }

    public void subscribeToEvents() {
        eventManager
            .GetEvent(RobotEventCollection.ToggleCollectorEvent.class)
            .Subscribe(
                pressed -> {
                    superstructure.setCollecting(pressed, true);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleCollectorReverseEvent.class)
            .Subscribe(
                pressed -> {
                    superstructure.setCollecting(pressed, false);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.UnlockClimberEvent.class)
            .Subscribe(
                climber::unlock
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleManualShootEvent.class)
            .Subscribe(
                () -> {
                    robotState.useManualShoot = !robotState.useManualShoot;
                    System.out.println("manual shooting toggled!");
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.ZeroPoseEvent.class)
            .Subscribe(
                () -> {
                    turret.setTurretAngle(Turret.kSouth);
                    drive.zeroSensors(Constants.kDefaultZeroingPose);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.BrakeModeEvent.class)
            .Subscribe(
                drive::setBraking
            );
        eventManager
            .GetEvent(RobotEventCollection.SlowModeEvent.class)
            .Subscribe(
                drive::setSlowMode
            );
        eventManager
            .GetEvent(RobotEventCollection.IncrementBucketEvent.class)
            .Subscribe(
                () -> {
                    distanceManager.incrementBucket(100);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.DecrementBucketEvent.class)
            .Subscribe(
                () -> {
                    distanceManager.incrementBucket(-100);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.AutoAimEvent.class)
            .Subscribe(
                aim -> {
                    if (aim) {
                        superstructure.autoAim();
                        turret.snapWithCamera();
                    } else {
                        superstructure.updatePoseWithCamera();
                        if (
                            robotState.defaultTurretControlMode ==
                                Turret.ControlMode.CENTER_FOLLOWING
                        ) {
                            turret.setFollowingAngle(Turret.kSouth);
                        }
                        turret.setControlMode(robotState.defaultTurretControlMode);
                    }
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleCameraEvent.class)
            .Subscribe(
                () -> {
                    robotState.overheating = !robotState.overheating;
                    System.out.println(
                        "overheating changed to = " + robotState.overheating
                    );
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.YeetShotEvent.class)
            .Subscribe(
                yeet -> {
                    if (robotState.useManualShoot) {
                        superstructure.setRevving(
                            yeet,
                            Shooter.TARMAC_TAPE_VEL,
                            true
                        ); // Tarmac
                    } else {
                        superstructure.setRevving(yeet, Shooter.NEAR_VELOCITY, true); // Barf shot
                    }
                    superstructure.setFiring(yeet);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.ShootEvent.class)
            .Subscribe(
                shooting -> {
                    superstructure.setRevving(
                        shooting,
                        Shooter.LAUNCHPAD_VEL,
                        robotState.useManualShoot
                    ); // Launchpad
                    superstructure.setFiring(shooting);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.TurretJogLeftEvent.class)
            .Subscribe(
                moving -> {
                    turret.setTurretSpeed(moving ? Turret.kJogSpeed : 0);
                    ledManager.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.TurretJogRightEvent.class)
            .Subscribe(
                moving -> {
                    turret.setTurretSpeed(moving ? -Turret.kJogSpeed : 0);
                    ledManager.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                }
            );
        eventManager
            .GetEvent(RobotEventCollection.ManualClimberArmUpEvent.class)
            .Subscribe(
                moving -> climber.setClimberPower(moving ? -.5 : 0)
            );
        eventManager
            .GetEvent(RobotEventCollection.ManualClimberArmDownEvent.class)
            .Subscribe(
                moving -> climber.setClimberPower(moving ? .5 : 0)
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleTopClampEvent.class)
            .Subscribe(
                climber::setTopClamp
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleTopClampEvent.class)
            .Subscribe(
                climber::setBottomClamp
            );
        eventManager
            .GetEvent(RobotEventCollection.ToggleTopClampEvent.class)
            .Subscribe(
                () -> {
                    if (climber.getCurrentStage() == 0) {
                        turret.setTurretAngle(Turret.kSouth);
                        superstructure.setStopped(true);
                    } else {
                        turret.setTurretAngle(Turret.kSouth - 30);
                    }

                    climber.incrementClimberStage();
                }
            );
    }
}
