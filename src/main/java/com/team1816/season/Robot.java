package com.team1816.season;

import static com.team1816.season.controlboard.ControlUtils.*;

import badlog.lib.BadLog;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.DrivetrainLogger;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.controlboard.ActionManager;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.*;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.SwerveDriveSignal;
import com.team254.lib.util.TimeDelayedBoolean;
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

    private final Looper mEnabledLooper = new Looper(this);
    private final Looper mDisabledLooper = new Looper(this);

    private IControlBoard mControlBoard;

    private final SubsystemManager mSubsystemManager;

    // subsystems
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final RobotState mRobotState;
    private final Drive mDrive;
    private final PowerDistribution pdh;
    private final PneumaticHub ph = new PneumaticHub(1); //use fatory.getPcm later
    private final Collector mCollector;
    private final Shooter mShooter;
    private final Turret mTurret;
    // private final Spinner spinner = Spinner.getInstance();
    private final Spindexer mSpindexer;
    private final Elevator mElevator;
    private final Orchestrator mOrchestrator;
    private final Climber mClimber;
    private final Camera mCamera;
    private final LedManager ledManager; //    private final Compressor compressor;

    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();

    private AutoModeSelector mAutoModeSelector;
    private AutoModeExecutor mAutoModeExecutor;
    private TrajectorySet trajectorySet;

    private boolean mDriveByCameraInAuto = false;
    private double loopStart;
    private boolean mHasBeenEnabled = false;

    private ActionManager actionManager;
    private AsyncTimer blinkTimer;

    // private PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Turret.ControlMode prevTurretControlMode = Turret.ControlMode.FIELD_FOLLOWING;

    Robot() {
        super();
        // initialize injector
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
        mDrive = (injector.getInstance(Drive.Factory.class)).getInstance();
        mTurret = injector.getInstance(Turret.class);
        mClimber = injector.getInstance(Climber.class);
        mCollector = injector.getInstance(Collector.class);
        mElevator = injector.getInstance(Elevator.class);
        mCamera = injector.getInstance(Camera.class);
        mSpindexer = injector.getInstance(Spindexer.class);
        mOrchestrator = injector.getInstance(Orchestrator.class);
        mShooter = injector.getInstance(Shooter.class);
        mRobotState = injector.getInstance(RobotState.class);
        mSuperstructure = injector.getInstance(Superstructure.class);
        mInfrastructure = injector.getInstance(Infrastructure.class);
        ledManager = injector.getInstance(LedManager.class);
        mSubsystemManager = injector.getInstance(SubsystemManager.class);
        mAutoModeSelector = injector.getInstance(AutoModeSelector.class);
        trajectorySet = injector.getInstance(TrajectorySet.class);
        //compressor =  new Compressor(getFactory().getPcmId(), PneumaticsModuleType.REVPH);
        pdh =
            new PowerDistribution(
                1,
                getFactory().getConstant("pdIsRev") == 0
                    ? PowerDistribution.ModuleType.kCTRE
                    : PowerDistribution.ModuleType.kRev
            );
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
            mControlBoard = injector.getInstance(IControlBoard.class);
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
                    String.valueOf(Constants.kPathFollowingMaxAccel)
                );

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    mEnabledLooper::getLastLoop,
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

                DrivetrainLogger.init(mDrive);
                if (RobotBase.isReal()) {
                    BadLog.createTopic("PDP/Current", "Amps", pdh::getTotalCurrent);

                    mDrive.CreateBadLogValue("Drivetrain PID", mDrive.pidToString());
                    mTurret.CreateBadLogValue("Turret PID", mTurret.pidToString());

                    BadLog.createTopic(
                        "Vision/DeltaXAngle",
                        "Degrees",
                        mCamera::getDeltaXAngle
                    );
                    BadLog.createTopic("Vision/Distance", "inches", mCamera::getDistance);
                    BadLog.createTopic(
                        "Vision/CenterX",
                        "pixels",
                        mCamera::getRawCenterX
                    );
                }
                mTurret.CreateBadLogTopic(
                    "Turret/ActPos",
                    "NativeUnits",
                    mTurret::getActualTurretPositionTicks,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/TargetPos",
                    "NativeUnits",
                    mTurret::getTargetPosition,
                    "hide",
                    "join:Turret/Positions"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/ErrorPos",
                    "NativeUnits",
                    mTurret::getPositionError
                );
                mTurret.CreateBadLogTopic(
                    "Turret/FieldToTurret",
                    "Degrees",
                    mRobotState::getLatestFieldToTurret,
                    "hide",
                    "join:Tracking/Angles"
                );
                mTurret.CreateBadLogTopic(
                    "Drive/HeadingRelativeToInitial",
                    "Degrees",
                    () -> mDrive.getHeadingRelativeToInitial().getDegrees(),
                    "hide",
                    "join:Tracking/Angles"
                );
                mTurret.CreateBadLogTopic(
                    "Turret/TurretAngle",
                    "Degrees",
                    mTurret::getActualTurretPositionDegrees,
                    "hide",
                    "join:Tracking/Angles"
                );
                mCamera.CreateBadLogTopic(
                    "Camera/CenterX",
                    "Degrees",
                    mCamera::getRawCenterX,
                    "hide"
                );
            }

            logger.finishInitialization();

            mSubsystemManager.setSubsystems(
                mDrive,
                mSuperstructure,
                mElevator,
                mSpindexer,
                //                mInfrastructure,
                mShooter,
                // spinner,
                mCollector,
                mOrchestrator,
                mTurret,
                mClimber,
                mCamera
            );

            mDrive.zeroSensors(Constants.StartingPose);
            //mTurret.zeroSensors();
            mClimber.zeroSensors();
            mOrchestrator.setStopped(true);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            ledManager.registerEnabledLoops(mEnabledLooper);
            ledManager.registerEnabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset();
            mDrive.setHeading(new Rotation2d());

            mAutoModeSelector.updateModeCreator();

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createHoldAction(mControlBoard::getSlowMode, mDrive::setSlowMode),
                    createHoldAction(
                        mControlBoard::getAutoAim,
                        pressed -> {
                            if (pressed) {
                                prevTurretControlMode = mTurret.getControlMode();
//                                mTurret.setControlMode(
//                                    Turret.ControlMode.CAMERA_FOLLOWING
//                                );
                            } else {
                                mTurret.setControlMode(prevTurretControlMode);
                            }
                        }
                    ),
//                    createHoldAction(
//                        mControlBoard::getRevShooter,
//                        revving -> {
//                            mOrchestrator.setRevving(revving, Shooter.MAX_VELOCITY);
//                            if (!revving) {
//                                mTurret.setControlMode(
//                                    Turret.ControlMode.FIELD_FOLLOWING
//                                );
//                                mShooter.setHood(true);
//                            }
//                        }
//                    ),
                    createHoldAction(
                        mControlBoard::getRevShooter,
                        revving -> {
                            mOrchestrator.setRevving(revving, Shooter.MID_VELOCITY);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        mOrchestrator::setFiring
                    ),
                    createAction( // make this an actual toggle?
                        mControlBoard::getCollectorToggle,
                        () -> {
                            mCollector.setState(Collector.COLLECTOR_STATE.COLLECTING);
                            mSpindexer.setSpindexer(0.5);
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getLowShoot,
                        lowShoot -> {
                            if (lowShoot) {
                                mSpindexer.setState(Spindexer.SPIN_STATE.FIRE);
                                mElevator.setState(Elevator.ELEVATOR_STATE.FIRE);
                                mShooter.setShooterNearVel();
                            } else {
                                mShooter.setVelocity(Shooter.COAST_VELOCIY);
                                mElevator.setState(Elevator.ELEVATOR_STATE.STOP);
                                mSpindexer.setState(Spindexer.SPIN_STATE.STOP);
                            }
                        }
                    ),
                    createAction(
                        mControlBoard::getMidShoot,
                        () -> {
                            mShooter.setShooterMidVel();
                        }
                    ),
                    createAction(
                        mControlBoard::getFarShoot,
                        () -> {
                            mShooter.setShooterFarVel();
                        }
                    ),
//                    createAction(
//                        mControlBoard::getLowPowerShoot,
//                        () -> {
//                            mShooter.setVelocity(2000);
//                        }
//                    ),
                    createHoldAction(
                        mControlBoard::getCollectorBackspin,
                        mOrchestrator::setFlushing
                    ),
                    createAction( // to turn the shooter on and off from its idle state - use at start of match
                        mControlBoard::getOrchestrator,
                        () -> mOrchestrator.setStopped()
                    ),
                    createAction(
                        mControlBoard::getFieldFollowing,
                        () -> {
                            if (mTurret.getControlMode() == Turret.ControlMode.FIELD_FOLLOWING) {
                                mTurret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING);
                            } else {
                                mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogLeft,
                        moving ->
                            mTurret.setTurretSpeed(moving ? -Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getTurretJogRight,
                        moving ->
                            mTurret.setTurretSpeed(moving ? Turret.TURRET_JOG_SPEED : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getClimberUp,
                        moving -> mClimber.setClimberPower(moving ? -.7 : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getClimberDown,
                        moving -> mClimber.setClimberPower(moving ? .7 : 0)
                    )
                );

            blinkTimer =
                new AsyncTimer(
                    3, // (3 s)
                    () -> ledManager.blinkStatus(LedManager.RobotStatus.ERROR),
                    () -> ledManager.indicateStatus(LedManager.RobotStatus.OFF)
                );
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            ledManager.setCameraLed(false);

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            //            mInfrastructure.setIs
            //       Control(false);

            mDisabledLooper.start();

            mDrive.stop();
            mDrive.setBrakeMode(false);

            mOrchestrator.setStopped(true);
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);
            Constants.StartingPose = mAutoModeExecutor.getAutoMode().getTrajectory().getInitialPose();

            // Robot starts where it's told for auto path
            mRobotState.reset();

            mHasBeenEnabled = true;

            mDrive.setOpenLoop(SwerveDriveSignal.NEUTRAL);

            mDrive.zeroSensors(Constants.StartingPose);
            //mTurret.zeroSensors();
            mClimber.zeroSensors();
            mOrchestrator.setStopped(false);

            mDrive.setControlState(Drive.DriveControlState.TRAJECTORY_FOLLOWING);

            mTurret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING);

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }
            // setHeading called already in both startTrajectory and zeroSensors
            mEnabledLooper.start();
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mDrive.setOpenLoop(SwerveDriveSignal.NEUTRAL);
            //mTurret.zeroSensors();
            mHasBeenEnabled = true;

            mEnabledLooper.start();
            mTurret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING);

            mCamera.setEnabled(getFactory().getConstant("useAutoAim") > 0);

            //System.out.println(mTurret.getActualTurretPositionTicks() + "+++++++"); // for debugging whether or not getActTicks works. doesn't seem to - ginget

            //            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();
        } catch (Throwable t) {
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

            mEnabledLooper.stop();
            mDisabledLooper.start();
            //mTurret.zeroSensors();
            mDrive.zeroSensors();
            blinkTimer.reset();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            System.out.println(t.getMessage());
        }
    }

    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mDrive.zeroSensors();
                mRobotState.reset();
                mDrive.setHeading(new Rotation2d());
                mDrive.setHeading(
                    mAutoModeSelector
                        .getAutoMode()
                        .get()
                        .getTrajectory()
                        .getInitialPose()
                        .getRotation()
                );
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                ledManager.indicateDefaultStatus();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                var auto = autoMode.get();
                System.out.println("Set auto mode to: " + auto.getClass().toString());
                mRobotState.field
                    .getObject("Trajectory");
                 mAutoModeExecutor.setAutoMode(auto);
            }
        } catch (Throwable t) {
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        boolean signalToResume = !mControlBoard.getDrivetrainFlipped(); // TODO: select auto interrupt button
        boolean signalToStop = mControlBoard.getDrivetrainFlipped();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            System.out.println("Auto mode interrupted ");
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl();
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
            manualControl(); // controls drivetrain and turret joystick calcs
        } catch (Throwable t) {
            throw t;
        }
        //System.out.println("CAMERA DISTANCE LINE 579 Robot.java" +  mCamera.getDistance());
        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    LatchedBoolean shouldChangeAzimuthSetpoint = new LatchedBoolean();

    public void manualControl() {
        // boolean arcadeDrive = false;
        actionManager.update();

        
        if (
            Math.abs(mControlBoard.getTurretXVal()) > 0.85 ||
            Math.abs(mControlBoard.getTurretYVal()) > 0.85
        ) {
            mTurret.setFollowingAngle(
                (
                    new Rotation2d(
                        mControlBoard.getTurretXVal(),
                        mControlBoard.getTurretYVal()
                    )
                ).getDegrees()
            );
        }

        mDrive.setTeleopInputs(
            mControlBoard.getThrottle(),
            mControlBoard.getStrafe(),
            mControlBoard.getTurn(),
            mControlBoard.getSlowMode(),
            /*mControlBoard.getFieldRelative()*/// Field Relative override button conflicts with collector
            false
        );
        //        }
    }

    @Override
    public void testPeriodic() {}
}
