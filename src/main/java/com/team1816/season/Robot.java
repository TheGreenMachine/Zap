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

    //State managers
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final RobotState mRobotState;

    // subsystems
    private final Drive mDrive;
    private final PowerDistribution pdh;
    private final PneumaticHub ph = new PneumaticHub(1); //use fatory.getPcm later
    private final Collector mCollector;
    private final Shooter mShooter;
    private final Turret mTurret;
    // private final Spinner spinner = Spinner.getInstance();
    private final Spindexer mSpindexer;
    private final Elevator mElevator;
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
    private boolean faulted;

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
        mSuperstructure = injector.getInstance(Superstructure.class);
        mShooter = injector.getInstance(Shooter.class);
        mRobotState = injector.getInstance(RobotState.class);
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
                    mShooter.CreateBadLogValue("Shooter PID", mShooter.pidToString());

                    if (mCamera.isImplemented()) {
                        BadLog.createTopic(
                            "Vision/DeltaXAngle",
                            "Degrees",
                            mCamera::getDeltaXAngle
                        );
                        BadLog.createTopic(
                            "Vision/Distance",
                            "inches",
                            mCamera::getDistance
                        );
                        BadLog.createTopic(
                            "Vision/CenterX",
                            "pixels",
                            mCamera::getRawCenterX
                        );
                    }
                }
                mShooter.CreateBadLogTopic(
                    "Shooter/ActVel",
                    "NativeUnits",
                    mShooter::getActualVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                mShooter.CreateBadLogTopic(
                    "Shooter/TargetVel",
                    "NativeUnits",
                    mShooter::getTargetVelocity,
                    "hide",
                    "join:Shooter/Velocities"
                );
                mShooter.CreateBadLogTopic(
                    "Shooter/Error",
                    "NativeUnits",
                    mShooter::getError,
                    "hide",
                    "join:Shooter/Velocities"
                );
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
                mElevator,
                mSpindexer,
                mShooter,
                mCollector,
                mTurret,
                mClimber,
                mCamera,
                ledManager
            );

            mDrive.zeroSensors();
            mTurret.zeroSensors();
            mClimber.zeroSensors();
            mSuperstructure.setStopped(true); // bool statement is for shooter state (stop or coast)

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
                                mTurret.setControlMode(
                                    Turret.ControlMode.CAMERA_FOLLOWING
                                );
                            } else {
                                mTurret.setControlMode(prevTurretControlMode); // this gets called when the robot inits - this could be bad?
                            }
                        }
                    ),
                    createHoldAction(
                        mControlBoard::getShoot,
                        shooting -> {
                            mSuperstructure.setRevving(shooting, Shooter.FAR_VELOCITY);
                            mSuperstructure.setFiring(shooting);
                        }
                    ),
                    createAction(
                        mControlBoard::getCollectorToggle,
                        () -> mSuperstructure.setCollecting(true)
                    ),
                    createAction(
                        mControlBoard::getCollectorBackspin,
                        () -> mSuperstructure.setCollecting(false)
                    ),
                    createAction(
                        mControlBoard::getZeroPose,
                        () -> {
                            mDrive.zeroSensors(Constants.ZeroPose);
                            mRobotState.reset(
                                Constants.ZeroPose,
                                Constants.EmptyRotation
                            );
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
                        moving -> mClimber.setClimberPower(moving ? -.3 : 0)
                    ),
                    createHoldAction(
                        mControlBoard::getClimberDown,
                        moving -> mClimber.setClimberPower(moving ? .3 : 0)
                    )//,
//                    createAction(
//                        mControlBoard::getTopClamp,
//                        mClimber::setTopClamp
//                    ),
//                    createAction(
//                        mControlBoard::getBottomClamp,
//                        mClimber::setBottomClamp
//                    ),
//                    createAction(
//                        mControlBoard::getIncrementClimberStage,
//                        mClimber::incrementClimberStage
//                    )
                );
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            mEnabledLooper.stop();

            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            ledManager.setCameraLed(false);

            mSuperstructure.setStopped(true);

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mDrive.stop();
            mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            mDisabledLooper.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);

            // Robot starts where it's told for auto path
            mRobotState.reset();

            mHasBeenEnabled = true;

            mDrive.zeroSensors();
            mTurret.zeroSensors();
            mClimber.zeroSensors();

            mSuperstructure.setStopped(false);

            mDrive.setControlState(Drive.DriveControlState.TRAJECTORY_FOLLOWING);

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }
            mEnabledLooper.start();
        }
        catch (Throwable t) {
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
            mTurret.zeroSensors();
            mHasBeenEnabled = true;

            mEnabledLooper.start();
            mTurret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING);

            mCamera.setEnabled(Constants.kUseVision); // do we enable here or only when we use vision? - this may cause an error b/c we enable more than once

            mSuperstructure.setStopped(false);

            mControlBoard.reset();
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

            mSuperstructure.setStopped(false);

            mEnabledLooper.stop();
            mDisabledLooper.start();
            mTurret.zeroSensors();
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
            faulted = true;
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
            faulted = true;
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
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
                }
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (
                autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()
            ) {
                var auto = autoMode.get();
                System.out.println("Set auto mode to: " + auto.getClass().toString());
                mRobotState.field.getObject("Trajectory");
                mAutoModeExecutor.setAutoMode(auto);
                Constants.StartingPose = auto.getTrajectory().getInitialPose();
                mRobotState.reset();
                mDrive.zeroSensors();
            }
        } catch (Throwable t) {
            faulted = true;
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
            false
        );
    }

    @Override
    public void testPeriodic() {}
}
