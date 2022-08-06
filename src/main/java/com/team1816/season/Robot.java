package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.*;

import badlog.lib.BadLog;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.LibModule;
import com.team1816.lib.controlboard.ControlBoardBrige;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.events.EventRegister;
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

    private BadLog logger;

    private final Injector injector;

    private final Looper enabledLoop = new Looper(this);
    private final Looper disabledLoop = new Looper(this);

    // controls
    private IControlBoard controlBoard;
    private ActionManager actionManager;
    private EventRegister eventManager;

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

    // autonomous
    private final AutoModeManager autoModeManager;

    // timing
    private double loopStart;

    // hack variables
    private boolean faulted;

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
        autoModeManager = injector.getInstance(AutoModeManager.class);
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
            eventManager.subscribeToControlBoard();
            DriverStation.silenceJoystickConnectionWarning(true);
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
                    turret::getActualPosTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                turret.CreateBadLogTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    turret::getDesiredPosTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                turret.CreateBadLogTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    turret::getPosError
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
            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);
            subsystemManager.zeroSensors();

            initializeActionManager();
            eventManager.subscribeToEvents();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    public void initializeActionManager() {
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
                        robotState.useManualShoot = !robotState.useManualShoot;
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
                    () -> {
                        distanceManager.incrementBucket(100);
                    }
                ),
                createAction(
                    () -> controlBoard.getAsBool("lowerBucket"),
                    () -> {
                        distanceManager.incrementBucket(-100);
                    }
                ),
                createHoldAction(
                    () -> controlBoard.getAsBool("autoAim"),
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
                ),
                createAction(
                    () -> controlBoard.getAsBool("toggleCamera"),
                    () -> {
                        robotState.overheating = !robotState.overheating;
                        System.out.println(
                            "overheating changed to = " + robotState.overheating
                        );
                    }
                ),
                createHoldAction(
                    () -> controlBoard.getAsBool("yeetShot"),
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
                ),
                createHoldAction(
                    () -> controlBoard.getAsBool("shoot"),
                    shooting -> {
                        superstructure.setRevving(
                            shooting,
                            Shooter.LAUNCHPAD_VEL,
                            robotState.useManualShoot
                        ); // Launchpad
                        superstructure.setFiring(shooting);
                    }
                ),
                createHoldAction(
                    () -> controlBoard.getAsBool("turretJogLeft"),
                    moving -> {
                        turret.setTurretSpeed(moving ? Turret.kJogSpeed : 0);
                        ledManager.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                    }
                ),
                createHoldAction(
                    () -> controlBoard.getAsBool("turretJogRight"),
                    moving -> {
                        turret.setTurretSpeed(moving ? -Turret.kJogSpeed : 0);
                        ledManager.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                    }
                ),
                createHoldAction( // climber up
                    () ->
                        controlBoard.getAsDouble("manualClimberArm") >
                        Constants.kJoystickBooleanThreshold,
                    moving -> climber.setClimberPower(moving ? -.5 : 0)
                ),
                createHoldAction( // climber down
                    () ->
                        controlBoard.getAsDouble("manualClimberArm") <
                        Constants.kJoystickBooleanThreshold,
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
                )
            );
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
        try {
            disabledLoop.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
            turret.zeroSensors();
            superstructure.setStopped(false);

            drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
            autoModeManager.startAuto();

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

            turret.zeroSensors();
            climber.zeroSensors();
            superstructure.setStopped(false);

            turret.setTurretAngle(Turret.kSouth);
            turret.setControlMode(robotState.defaultTurretControlMode);

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

            if (ControlBoardBrige.getInstance().isDemoMode()) {
                controlBoard.outputToSmartDashboard();
            }
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
            if (ControlBoardBrige.getInstance().isDemoMode()) {
                controlBoard.update();
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

        // Optional functionality making shooter always rev to velocity needed to score based on predicted position on field
        if (
            !controlBoard.getAsBool("shoot") &&
            !controlBoard.getAsBool("yeetShot") &&
            Constants.kUsePoseTrack
        ) {
            superstructure.setRevving(true, -1, true);
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
