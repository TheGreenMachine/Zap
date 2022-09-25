package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.createAction;
import static com.team1816.season.controlboard.ControlUtils.createHoldAction;

import com.team1816.lib.BadLogger;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.controlboard.ControlBoardBrige;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Robot extends TimedRobot {

    private final Looper enabledLoop;
    private final Looper disabledLoop;

    // controls
    private IControlBoard controlBoard;
    private ActionManager actionManager;

    private final Infrastructure infrastructure;
    private final SubsystemManager subsystemManager;

    //State managers
    private final Superstructure superstructure;
    private final RobotState robotState;

    // subsystems
    private final Drive drive;
    private final Collector collector;
    private final Shooter shooter;
    private final Turret turret;
    private final Spindexer spindexer;
    private final Elevator elevator;
    private final Climber climber;
    private final Cooler cooler;
    private final Camera camera;
    private final LedManager ledManager;
    private final DistanceManager distanceManager;

    private static RobotFactory factory;

    // autonomous
    private final AutoModeManager autoModeManager;

    // timing
    private double loopStart;

    // hack variables
    private final Turret.ControlMode defaultTurretControlMode =
        Turret.ControlMode.POSITION;
    private boolean faulted;
    private boolean useManualShoot = false;

    Robot() {
        super();
        // initialize injector
        Injector.registerModule(new SeasonModule());
        enabledLoop = new Looper(this);
        disabledLoop = new Looper(this);
        drive = (Injector.get(Drive.Factory.class)).getInstance(); //TODO: need to fix this get drive instance should just return the proper one
        turret = Injector.get(Turret.class);
        climber = Injector.get(Climber.class);
        collector = Injector.get(Collector.class);
        elevator = Injector.get(Elevator.class);
        camera = Injector.get(Camera.class);
        spindexer = Injector.get(Spindexer.class);
        superstructure = Injector.get(Superstructure.class);
        infrastructure = Injector.get(Infrastructure.class);
        shooter = Injector.get(Shooter.class);
        cooler = Injector.get(Cooler.class);
        robotState = Injector.get(RobotState.class);
        distanceManager = Injector.get(DistanceManager.class);
        ledManager = Injector.get(LedManager.class);
        subsystemManager = Injector.get(SubsystemManager.class);
        autoModeManager = Injector.get(AutoModeManager.class);
    }

    public static RobotFactory getFactory() {
        if (factory == null) factory = Injector.get(RobotFactory.class);
        return factory;
    }

    public Double getLastRobotLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    public Double getLastEnabledLoop() {
        return enabledLoop.getLastLoop();
    }

    @Override
    public void robotInit() {
        try {
            /** register all subsystems */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            subsystemManager.setSubsystems(
                drive,
                elevator,
                spindexer,
                shooter,
                collector,
                turret,
                climber,
                camera,
                ledManager,
                cooler
            );
            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);
            subsystemManager.zeroSensors();

            // register badlogs
            if (Constants.kIsBadlogEnabled) {
                BadLogger.setupLogs(this);
                subsystemManager.createLogs();
            }

            // register controllers
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            actionManager =
                new ActionManager(
                    createHoldAction(
                        () -> controlBoard.getAsBool("toggleCollector"),
                        pressed -> superstructure.setCollecting(pressed, true)
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("toggleCollectorReverse"),
                        pressed -> superstructure.setCollecting(pressed, false)
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("unlockClimber"),
                        climber::unlock
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleManualShoot"),
                        () -> {
                            useManualShoot = !useManualShoot;
                            System.out.println("manual shooting toggled!");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("zeroPose"),
                        () -> {
                            turret.setTurretAngle(Turret.kSouth);
                            drive.zeroSensors(Constants.kDefaultZeroingPose);
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("brakeMode"),
                        drive::setBraking
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("slowMode"),
                        drive::setSlowMode
                    ),
                    // Operator Gamepad
                    createAction(
                        () -> controlBoard.getAsBool("raiseBucket"),
                        () -> distanceManager.incrementBucket(100)
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("lowerBucket"),
                        () -> distanceManager.incrementBucket(-100)
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("autoAim"),
                        aim -> {
                            if (aim) {
                                superstructure.autoAim();
                                turret.snap();
                            } else {
                                superstructure.updatePoseWithCamera();
                                turret.setControlMode(defaultTurretControlMode);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleCamera"),
                        () -> {
                            camera.setCameraEnabled(!camera.isEnabled());
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("yeetShot"),
                        yeet -> {
                            if (useManualShoot) {
                                superstructure.setRevving(
                                    yeet,
                                    Shooter.TARMAC_TAPE_VEL,
                                    true
                                ); // Tarmac
                            } else {
                                superstructure.setRevving(
                                    yeet,
                                    Shooter.NEAR_VELOCITY,
                                    true
                                ); // Low
                            }
                            superstructure.setFiring(yeet);
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("shoot"),
                        shooting -> {
                            superstructure.setRevving(
                                shooting,
                                Shooter.LAUNCHPAD_VEL,
                                useManualShoot
                            ); // Launchpad
                            superstructure.setFiring(shooting);
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("turretJogLeft"),
                        moving -> {
                            turret.setTurretSpeed(moving ? Turret.kJogSpeed : 0);
                            ledManager.indicateStatus(
                                LedManager.RobotStatus.MANUAL_TURRET
                            );
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("turretJogRight"),
                        moving -> {
                            turret.setTurretSpeed(moving ? -Turret.kJogSpeed : 0);
                            ledManager.indicateStatus(
                                LedManager.RobotStatus.MANUAL_TURRET
                            );
                        }
                    ),
                    createHoldAction( // climber up
                        () ->
                            controlBoard.getAsDouble("manualClimberArm") >
                            Controller.kJoystickBooleanThreshold,
                        moving -> climber.setClimberPower(moving ? -.5 : 0)
                    ),
                    createHoldAction( // climber down
                        () ->
                            controlBoard.getAsDouble("manualClimberArm") <
                            Controller.kJoystickBooleanThreshold,
                        moving -> climber.setClimberPower(moving ? .5 : 0)
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleTopClamp"),
                        climber::setTopClamp
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleBottomClamp"),
                        climber::setBottomClamp
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoClimb"),
                        () -> {
                            if (climber.getCurrentStage() == 0) {
                                turret.setTurretAngle(Turret.kSouth);
                                superstructure.setStopped(true);
                            } else {
                                turret.setTurretAngle(Turret.kSouth - 30);
                            }

                            climber.incrementClimberStage();
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleTurretMode"),
                        turret::revolve
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("eject"),
                        eject -> {
                            if (
                                eject &&
                                !turret.getControlMode().equals(Turret.ControlMode.EJECT)
                            ) {
                                turret.setControlMode(Turret.ControlMode.EJECT);
                            } else {
                                turret.revolve();
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("littleMan"),
                        () -> {
                            superstructure.setSuperstructureState(
                                Superstructure.STATE.LITTLE_MAN
                            );
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("fatBoy"),
                        () -> {
                            superstructure.setSuperstructureState(
                                Superstructure.STATE.FAT_BOY
                            );
                        }
                    )
                );
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            enabledLoop.stop();

            // Stop any running autos
            autoModeManager.stopAuto();
            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            camera.setCameraEnabled(false);

            if (autoModeManager.getSelectedAuto() == null) {
                autoModeManager.reset();
            }

            superstructure.setStopped(true);
            subsystemManager.stop();

            robotState.resetAllStates();
            cooler.zeroSensors();
            drive.zeroSensors();

            disabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        disabledLoop.stop();
        ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

        drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
        turret.zeroSensors();
        superstructure.setStopped(false);

        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        autoModeManager.startAuto();

        enabledLoop.start();
    }

    @Override
    public void teleopInit() {
        try {
            disabledLoop.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            turret.zeroSensors();
            climber.zeroSensors();
            superstructure.setStopped(false);

            turret.setTurretAngle(Turret.kSouth);
            turret.setControlMode(defaultTurretControlMode);

            infrastructure.startCompressor();

            enabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writeToHardware();
            }

            superstructure.setStopped(false);

            enabledLoop.stop();
            disabledLoop.start();
            turret.zeroSensors();
            drive.zeroSensors();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (subsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            // update shuffleboard for subsystem values
            subsystemManager.outputToSmartDashboard();
            // update robot state on field for Field2D widget
            robotState.outputToSmartDashboard();
            // update shuffleboard selected auto mode
            autoModeManager.outputToSmartDashboard();
        } catch (Throwable t) {
            faulted = true;
            System.out.println(t.getMessage());
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton()) {
                drive.zeroSensors(Constants.kDefaultZeroingPose);
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
            }

            // Periodically check if drivers changed desired auto - if yes, then update the robot's position on the field
            if (autoModeManager.update()) {
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
                robotState.field
                    .getObject("Trajectory")
                    .setTrajectory(
                        autoModeManager.getSelectedAuto().getCurrentTrajectory()
                    );
            }

            // check if demo mode speed multiplier changed
            if (drive.isDemoMode()) { //todo: should be using injector
                drive.update();
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        robotState.field
            .getObject("Trajectory")
            .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());

        if (Constants.kIsLoggingAutonomous) {
            BadLogger.update();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl(); // controls drivetrain and turret joystick control mode
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
        if (Constants.kIsLoggingTeleOp) {
            BadLogger.update();
        }
    }

    public void manualControl() {
        // update what's currently being imputed from both driver and operator controllers
        actionManager.update();

        // Field-relative controls for turret (ie: left on joystick makes turret point left on the field, instead of left relative to the robot)
        if (
            Math.abs(controlBoard.getAsDouble("manualTurretXVal")) > 0.90 ||
            Math.abs(controlBoard.getAsDouble("manualTurretYVal")) > 0.90
        ) {
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            turret.setFollowingAngle(
                (
                    new Rotation2d(
                        (-1) * controlBoard.getAsDouble("manualTurretYVal"),
                        (-1) * controlBoard.getAsDouble("manualTurretXVal")
                    )
                ).getDegrees()
            );
        }

        drive.setTeleopInputs(
            -controlBoard.getAsDouble("throttle"),
            -controlBoard.getAsDouble("strafe"),
            controlBoard.getAsDouble("rotation")
        );
    }

    @Override
    public void testPeriodic() {}
}
