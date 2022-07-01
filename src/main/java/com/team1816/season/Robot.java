package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.*;

import badlog.lib.BadLog;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.LibModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.auto.AutoModeSelector;
import com.team1816.season.auto.paths.TrajectorySet;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class Robot extends TimedRobot {

    private BadLog logger;

    private final Injector injector;

    private final Looper enabledLoop = new Looper(this);
    private final Looper disabledLoop = new Looper(this);

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

    // debugging / testing
    private final LatchedBoolean wantExecution = new LatchedBoolean();
    private final LatchedBoolean wantInterrupt = new LatchedBoolean();

    // autonomous
    private final AutoModeSelector autoModeSelector;
    private final AutoModeExecutor autoModeExecutor;
    private TrajectorySet trajectorySet;

    // timing
    private double loopStart;

    // hack variables
    private final Turret.ControlMode defaultTurretControlMode =
        Turret.ControlMode.FIELD_FOLLOWING;
    private boolean faulted;
    private boolean useManualShoot = false;

    Robot() {
        super();
        // initialize injector
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
        drive = (injector.getInstance(Drive.Factory.class)).getInstance();
        turret = injector.getInstance(Turret.class);
        climber = injector.getInstance(Climber.class);
        collector = injector.getInstance(Collector.class);
        elevator = injector.getInstance(Elevator.class);
        camera = injector.getInstance(Camera.class);
        spindexer = injector.getInstance(Spindexer.class);
        superstructure = injector.getInstance(Superstructure.class);
        infrastructure = injector.getInstance(Infrastructure.class);
        shooter = injector.getInstance(Shooter.class);
        cooler = injector.getInstance(Cooler.class);
        robotState = injector.getInstance(RobotState.class);
        distanceManager = injector.getInstance(DistanceManager.class);
        ledManager = injector.getInstance(LedManager.class);
        subsystemManager = injector.getInstance(SubsystemManager.class);
        autoModeSelector = injector.getInstance(AutoModeSelector.class);
        autoModeExecutor = injector.getInstance(AutoModeExecutor.class);
        trajectorySet = injector.getInstance(TrajectorySet.class);
    }

    public static RobotFactory getFactory() {
        return RobotFactory.getInstance();
    }

    private Double getLastLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    @Override
    public void robotInit() {
        try {
            controlBoard = injector.getInstance(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);
            if (Constants.kIsBadlogEnabled) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                // if there is a usb drive use it
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

                BadLog.createValue(
                    "Max Velocity",
                    String.valueOf(Constants.kPathFollowingMaxVelMeters)
                );
                BadLog.createValue(
                    "Max Acceleration",
                    String.valueOf(Constants.kPathFollowingMaxAccelMeters)
                );

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    enabledLoop::getLastLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastLoop,
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

                DrivetrainLogger.init(drive);
                if (RobotBase.isReal()) {
                    BadLog.createTopic(
                        "PDP/Current",
                        "Amps",
                        infrastructure.getPd()::getTotalCurrent
                    );

                    drive.CreateBadLogValue("Drivetrain PID", drive.pidToString());
                    turret.CreateBadLogValue("Turret PID", turret.pidToString());
                    shooter.CreateBadLogValue("Shooter PID", shooter.pidToString());

                    if (camera.isImplemented()) {
                        BadLog.createTopic(
                            "Vision/DeltaXAngle",
                            "Degrees",
                            camera::getDeltaX
                        );
                        BadLog.createTopic(
                            "Vision/Distance",
                            "inches",
                            camera::getDistance
                        );
                    }
                }
                shooter.CreateBadLogTopic(
                    "Shooter/ActVel",
                    "NativeUnits",
                    shooter::getActualVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                shooter.CreateBadLogTopic(
                    "Shooter/TargetVel",
                    "NativeUnits",
                    shooter::getTargetVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                shooter.CreateBadLogTopic(
                    "Shooter/Error",
                    "NativeUnits",
                    shooter::getError,
                    "hide",
                    "join:Shooter/Velocities"
                );
                turret.CreateBadLogTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    turret::getActualTurretPositionTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                turret.CreateBadLogTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    turret::getTargetPosition,
                    "hide",
                    "join:Turret/Positions"
                );
                turret.CreateBadLogTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    turret::getPositionError
                );
            }

            logger.finishInitialization();

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

            subsystemManager.zeroSensors();
            superstructure.setStopped(true); // bool statement is for shooter state (stop or coast)
            distanceManager.outputBucketOffsets();

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);

            // Robot starts forwards.
            robotState.resetPosition();

            autoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager(
                    createHoldAction(
                        controlBoard::getCollectorToggle,
                        pressed -> superstructure.setCollecting(pressed, true)
                    ),
                    createHoldAction(
                        controlBoard::getCollectorBackspin,
                        pressed -> superstructure.setCollecting(pressed, false)
                    ),
                    createAction(controlBoard::getUnlockClimber, climber::unlock),
                    createAction(
                        controlBoard::getUseManualShoot,
                        () -> {
                            useManualShoot = !useManualShoot;
                            System.out.println("manual shooting toggled!");
                        }
                    ),
                    createAction(
                        controlBoard::getZeroPose,
                        () -> {
                            turret.setTurretAngle(Turret.SOUTH);
                            drive.zeroSensors(Constants.ZeroPose);
                        }
                    ),
                    createHoldAction(controlBoard::getBrakeMode, drive::setBraking),
                    createHoldAction(controlBoard::getSlowMode, drive::setSlowMode),
                    // Operator Gamepad
                    createAction(
                        controlBoard::getRaiseBucket,
                        () -> distanceManager.incrementBucket(100)
                    ),
                    createAction(
                        controlBoard::getLowerBucket,
                        () -> distanceManager.incrementBucket(-100)
                    ),
                    createHoldAction(
                        controlBoard::getAutoAim,
                        aim -> {
                            if (aim) {
                                turret.snapWithCamera();
                            } else {
                                superstructure.updatePoseWithCamera();
                                if (
                                    defaultTurretControlMode ==
                                    Turret.ControlMode.CENTER_FOLLOWING
                                ) {
                                    turret.setFollowingAngle(Turret.SOUTH);
                                }
                                turret.setControlMode(defaultTurretControlMode);
                            }
                        }
                    ),
                    createAction(
                        controlBoard::getCameraToggle,
                        () -> {
                            robotState.overheating = !robotState.overheating;
                            System.out.println(
                                "overheating changed to = " + robotState.overheating
                            );
                        }
                    ),
                    createHoldAction(
                        controlBoard::getYeetShot,
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
                                ); // Barf shot
                            }
                            superstructure.setFiring(yeet);
                        }
                    ),
                    createHoldAction(
                        controlBoard::getShoot,
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
                        controlBoard::getTurretJogLeft,
                        moving -> turret.setTurretSpeed(moving ? Turret.JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        controlBoard::getTurretJogRight,
                        moving -> turret.setTurretSpeed(moving ? -Turret.JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        controlBoard::getClimberUp,
                        moving -> climber.setClimberPower(moving ? -.5 : 0)
                    ),
                    createHoldAction(
                        controlBoard::getClimberDown,
                        moving -> climber.setClimberPower(moving ? .5 : 0)
                    ),
                    createAction(controlBoard::getTopClamp, climber::setTopClamp),
                    createAction(controlBoard::getBottomClamp, climber::setBottomClamp),
                    createAction(
                        controlBoard::getAutoClimb,
                        () -> {
                            if (climber.getCurrentStage() == 0) {
                                turret.setTurretAngle(Turret.SOUTH);
                                superstructure.setStopped(true);
                            } else {
                                turret.setTurretAngle(Turret.SOUTH - 30);
                            }

                            climber.incrementClimberStage();
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

            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            camera.setCameraEnabled(false);

            superstructure.setStopped(true);
            subsystemManager.stop();

            robotState.resetAllStates();
            cooler.zeroSensors();
            drive.zeroSensors();

            // Reset all auto mode states.
            if (autoModeExecutor != null) {
                autoModeExecutor.stop();
            }
            autoModeSelector.updateModeCreator();

            disabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            disabledLoop.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts at first waypoint (Pose2D) of current auto path chosen
            robotState.resetPosition();

            drive.zeroSensors();
            turret.zeroSensors();

            superstructure.setStopped(false);

            drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
            autoModeExecutor.start();

            enabledLoop.start();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            disabledLoop.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            if (autoModeExecutor != null) {
                autoModeExecutor.stop();
            }

            turret.zeroSensors();
            climber.zeroSensors();

            enabledLoop.start();

            turret.setTurretAngle(Turret.SOUTH);
            turret.setControlMode(defaultTurretControlMode);

            superstructure.setStopped(false);
            infrastructure.startCompressor();
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
            autoModeSelector.outputToSmartDashboard();
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
                drive.zeroSensors(Constants.ZeroPose);
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
            }

            // Periodically check if drivers changed desired auto - if yes, then update the actual auto mode
            autoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = autoModeSelector.getAutoMode();
            if (
                autoMode.isPresent() && autoMode.get() != autoModeExecutor.getAutoMode()
            ) {
                var auto = autoMode.get();
                System.out.println("Set auto mode to: " + auto.getClass().toString());
                robotState.field.getObject("Trajectory");
                autoModeExecutor.setAutoMode(auto);
                Constants.StartingPose = auto.getTrajectory().getInitialPose();
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        // Debugging functionality to stop autos mid-path - Currently not in use
        boolean signalToResume = !controlBoard.getUnlockClimber();
        boolean signalToStop = controlBoard.getUnlockClimber();
        if (autoModeExecutor.isInterrupted()) {
            manualControl();

            if (wantExecution.update(signalToResume)) {
                autoModeExecutor.resume();
            }
        }
        if (wantInterrupt.update(signalToStop)) {
            autoModeExecutor.interrupt();
        }

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
            Math.abs(controlBoard.getTurretXVal()) > 0.90 ||
            Math.abs(controlBoard.getTurretYVal()) > 0.90
        ) {
            turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
            turret.setFollowingAngle(
                (
                    new Rotation2d(
                        controlBoard.getTurretXVal(),
                        controlBoard.getTurretYVal()
                    )
                ).getDegrees()
            );
        }

        // Optional functionality making shooter always rev to velocity needed to score based on predicted position on field
        if (
            !controlBoard.getShoot() &&
            !controlBoard.getYeetShot() &&
            Constants.kUsePoseTrack
        ) {
            superstructure.setRevving(true, -1, true);
        }

        drive.setTeleopInputs(
            controlBoard.getThrottle(),
            controlBoard.getStrafe(),
            controlBoard.getTurn()
        );
    }

    @Override
    public void testPeriodic() {}
}
