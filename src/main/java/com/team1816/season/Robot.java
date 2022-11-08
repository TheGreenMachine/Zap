package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.createAction;
import static com.team1816.season.controlboard.ControlUtils.createHoldAction;

import badlog.lib.BadLog;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.DrivetrainLogger;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
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

    private static BadLog logger;

    // controls
    private IControlBoard controlBoard;
    private ActionManager actionManager;

    private final Infrastructure infrastructure;
    private final SubsystemManager subsystemManager;

    //State managers
    private final Orchestrator orchestrator;
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
        Turret.ControlMode.ABSOLUTE_FOLLOWING;
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
        orchestrator = Injector.get(Orchestrator.class);
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
            // register all subsystems
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

            /** Register BadLogs */
            if (Constants.kIsBadlogEnabled) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                // if there is a USB drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    logFileDir = "/media/sda1/";
                }
                if (RobotBase.isSimulation()) {
                    if (System.getProperty("os.name").toLowerCase().contains("win")) {
                        logFileDir = System.getenv("temp") + "\\";
                    } else {
                        logFileDir = System.getProperty("user.dir") + "/";
                    }
                }
                var filePath = logFileDir + robotName + "_" + logFile + ".bag";
                logger = BadLog.init(filePath);

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    this::getLastEnabledLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastRobotLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/Timestamp",
                    "s",
                    Timer::getFPGATimestamp,
                    "xaxis",
                    "hide"
                );
                BadLog.createTopic(
                    "Vision/Distance",
                    "inches",
                    robotState::getDistanceToGoal
                );
                BadLog.createValue("Drivetrain PID", drive.pidToString());
                DrivetrainLogger.init(drive);
                BadLog.createValue("Shooter PID", shooter.pidToString());
                BadLog.createTopic(
                    "Shooter/ActVel",
                    "NativeUnits",
                    shooter::getActualVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                BadLog.createTopic(
                    "Shooter/TargetVel",
                    "NativeUnits",
                    shooter::getTargetVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                BadLog.createTopic(
                    "Shooter/Error",
                    "NativeUnits",
                    shooter::getError,
                    "hide",
                    "join:Shooter/Velocities"
                );
                BadLog.createValue("Turret PID", turret.pidToString());
                BadLog.createTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    turret::getActualPosTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                BadLog.createTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    turret::getDesiredPosTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                BadLog.createTopic("Turret/ErrorPos", "NativeUnits", turret::getPosError);
                BadLog.createTopic(
                    "PDP/Current",
                    "Amps",
                    infrastructure.getPd()::getTotalCurrent
                );

                BadLog.createTopic(
                    "Pigeon/AccelerationX",
                    "G",
                    infrastructure::getXAcceleration
                );
                BadLog.createTopic(
                    "Pigeon/AccelerationY",
                    "G",
                    infrastructure::getYAcceleration
                );
                BadLog.createTopic(
                    "Pigeon/AccelerationZ",
                    "G",
                    infrastructure::getZAcceleration
                );
                logger.finishInitialization();
            }
            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);
            subsystemManager.zeroSensors();

            /** register controlboard */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createHoldAction(
                        () -> controlBoard.getAsBool("toggleCollector"),
                        pressed -> orchestrator.setCollecting(pressed, true)
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("toggleCollectorReverse"),
                        pressed -> orchestrator.setCollecting(pressed, false)
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
                        () -> controlBoard.getAsBool("eject"),
                        aim -> {
                            if (aim) {
                                orchestrator.autoAim();
                                turret.snap();
                            } else {
                                orchestrator.updatePoseWithCamera();
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
                                orchestrator.setRevving(
                                    yeet,
                                    Shooter.TARMAC_TAPE_VEL,
                                    true
                                ); // Tarmac
                            } else {
                                orchestrator.setRevving(
                                    yeet,
                                    Shooter.NEAR_VELOCITY,
                                    true
                                ); // Low
                            }
                            orchestrator.setFiring(yeet);
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("shoot"),
                        shooting -> {
                            orchestrator.setRevving(
                                shooting,
                                Shooter.LAUNCHPAD_VEL,
                                useManualShoot
                            ); // Launchpad
                            orchestrator.setFiring(shooting);
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
                                orchestrator.setStopped(true);
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
                        () -> controlBoard.getAsBool("autoAim"),
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
                            orchestrator.setSuperstructureState(
                                Orchestrator.STATE.LITTLE_MAN
                            );
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("fatBoy"),
                        () -> {
                            orchestrator.setSuperstructureState(
                                Orchestrator.STATE.FAT_BOY
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

            orchestrator.setStopped(true);
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
        orchestrator.setStopped(false);

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
            orchestrator.setStopped(false);

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

            orchestrator.setStopped(false);

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
            logger.updateTopics();
            logger.log();
        }
    }

    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl(); // controls drivetrain and turret joystick control mode
            //            if (orchestrator.needsVisionUpdate() || !robotState.isPoseUpdated) { //TODO Future function, uncomment later
            //                robotState.isPoseUpdated = false;
            //                orchestrator.calculatePoseFromCamera();
            //            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
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
