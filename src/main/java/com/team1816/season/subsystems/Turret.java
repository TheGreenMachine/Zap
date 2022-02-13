package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class Turret extends Subsystem implements PidProvider {

    private static final String NAME = "turret";

    public static final double TURRET_JOG_SPEED = 0.25;

    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_EAST = 270; // deg

    public static final int TURRET_LIMIT_REVERSE =
        ((int) factory.getConstant(NAME, "revLimit"));
    public static final int TURRET_LIMIT_FORWARD =
        ((int) factory.getConstant(NAME, "fwdLimit"));

    public static final int ABS_TICKS_SOUTH =
        ((int) factory.getConstant(NAME, "absPosTicksSouth"));
    public static final double MAX_ANGLE = convertTurretTicksToDegrees(
        TURRET_LIMIT_FORWARD - TURRET_LIMIT_REVERSE
    );

    // Constants
    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 0;
    private static final int TURRET_ENCODER_PPR = (int) factory.getConstant(
        NAME,
        "encPPR"
    );
    private static final int TURRET_ENCODER_MASK = TURRET_ENCODER_PPR - 1;
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    // Components
    @Inject
    private static Camera camera;

    @Inject
    private static RobotState robotState;

    @Inject
    private static LedManager led;

    private final IMotorControllerEnhanced turret;

    private final String pidSlot = "slot0";
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    // Turret State Tracking
    private int desiredTurretPos = 0;
    private int followingTurretPos = 0;
    private double turretSpeed;
    private boolean outputsChanged;
    private double turretAngleRelativeToField;
    private ControlMode controlMode = ControlMode.MANUAL;

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");

        turret.setNeutralMode(NeutralMode.Brake);

        SmartDashboard.putNumber("TURRET_POSITION_MIN", TURRET_LIMIT_REVERSE);
        SmartDashboard.putNumber("TURRET_POSITION_MAX", TURRET_LIMIT_FORWARD);
        PIDSlotConfiguration pidConfig = factory.getPidSlotConfig(NAME, pidSlot);
        this.kP = pidConfig.kP;
        this.kI = pidConfig.kI;
        this.kD = pidConfig.kD;
        this.kF = pidConfig.kF;
        synchronized (this) {
            this.zeroSensors();

            // Position Control
            double peakOutput = 0.75;

            turret.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
            turret.configNominalOutputForward(0, Constants.kCANTimeoutMs);
            turret.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
            turret.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
            turret.configAllowableClosedloopError(
                kPIDGyroIDx,
                ALLOWABLE_ERROR_TICKS,
                Constants.kCANTimeoutMs
            );
            turret.configAllowableClosedloopError(
                kPIDVisionIDx,
                ALLOWABLE_ERROR_TICKS,
                Constants.kCANTimeoutMs
            );

            // Soft Limits
            turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
            turret.configForwardSoftLimitThreshold(
                TURRET_LIMIT_FORWARD,
                Constants.kCANTimeoutMs
            ); // Forward = MAX
            turret.configReverseSoftLimitThreshold(
                TURRET_LIMIT_REVERSE,
                Constants.kCANTimeoutMs
            ); // Reverse = MIN
            turret.overrideLimitSwitchesEnable(true);
            turret.overrideSoftLimitsEnable(true);
            //            turretAngleRelativeToField =
            //                robotState.getHeadingRelativeToInitial().getDegrees();
        }
    }

    // methods used externally to move the turret - change control mode
    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretPosition(convertTurretDegreesToTicks(angle));
    }

    public void setTurretSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        if (turretSpeed != speed) {
            turretSpeed = speed;
            outputsChanged = true;
        }
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualTurretPositionDegrees());
    }

    @Override
    public void readPeriodicInputs() {
        turretAngleRelativeToField = robotState.getLatestFieldToTurret();
        if (RobotBase.isSimulation()) {
            //show turret
            var robotPose = robotState.field_to_vehicle;
            var turret = robotState.field.getObject("turret");
            turret.setPose(
                robotPose.getX(),
                robotPose.getY(),
                Rotation2d.fromDegrees(turretAngleRelativeToField)
            );
        }
    }

    // calls internal methods based on control state
    @Override
    public void writePeriodicOutputs() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl(followingTurretPos);
                break;
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl(followingTurretPos);
                break;
            case POSITION:
                positionControl(desiredTurretPos);
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    // internal methods periodically called by writePeriodic
    private void autoHome() {
        var angle = camera.getDeltaXAngle();
        int adj =
            convertTurretDegreesToTicks(angle * .10) +
            followingTurretPos -
            ABS_TICKS_SOUTH;
        //        System.out.println(angle + " " + adj + " " + followingTurretPos);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void trackGyro() {
        int fieldTickOffset =
            convertTurretDegreesToTicks(turretAngleRelativeToField) - ABS_TICKS_SOUTH;
        int adj = desiredTurretPos + fieldTickOffset;
        // Valid positions are 0 to encoder max ticks if we go negative adjust
        //if (adj < 0) adj += TURRET_ENCODER_PPR;
        adj = -(int) (Math.abs(adj) % Math.abs(TURRET_ENCODER_MASK));
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl(double rawPos) {
        if (outputsChanged) {
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, rawPos);
            double result = turret.getClosedLoopTarget(0);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            if (turretSpeed == 0) {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.Position,
                    getActualTurretPositionTicks() + 200 * turret.getMotorOutputPercent()
                );
            } else {
                turret.set(
                    com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                    turretSpeed
                );
            }
            outputsChanged = false;
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            if (controlMode == ControlMode.CAMERA_FOLLOWING) {
                if (Constants.kUseAutoAim) {
                    turret.selectProfileSlot(kPIDVisionIDx, 0);
                    this.controlMode = controlMode;
                    camera.setEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                }
            } else {
                turret.selectProfileSlot(kPIDGyroIDx, 0);
                this.controlMode = controlMode;
                camera.setEnabled(false);
                if (controlMode == ControlMode.MANUAL) {
                    led.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                } else {
                    led.indicateDefaultStatus();
                }
            }
        }
    }

    // internal conversions and turret value getters
    private synchronized void setTurretPosition(double position) {
        //Since we are using position we need ensure value stays in one rotation
        int adjPos = -(int) (Math.abs(position) % Math.abs(TURRET_ENCODER_MASK));
        if (desiredTurretPos != adjPos) {
            desiredTurretPos = adjPos;
            outputsChanged = true;
        }
    }

    public static int convertTurretDegreesToTicks(double degrees) {
        return (int) (((degrees) / 360.0) * TURRET_ENCODER_PPR) + ABS_TICKS_SOUTH;
    }

    public static double convertTurretTicksToDegrees(double ticks) {
        var adjTicks = (ticks - ABS_TICKS_SOUTH);
        return adjTicks / TURRET_ENCODER_PPR * 360;
    }

    public double getActualTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getActualTurretPositionTicks());
    }

    public double getActualTurretPositionTicks() {
        return turret.getSelectedSensorPosition(kPrimaryCloseLoop);
    }

    public double getTargetPosition() {
        return followingTurretPos;
    }

    public double getPositionError() {
        return turret.getClosedLoopError(kPrimaryCloseLoop);
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    // calls used during initialization || game phase change
    @Override
    public synchronized void zeroSensors() {
        if (turret instanceof TalonSRX) {
            var sensors = ((TalonSRX) turret).getSensorCollection();
            var sensorVal = sensors.getPulseWidthPosition() & TURRET_ENCODER_MASK;
            sensors.setQuadraturePosition(sensorVal, Constants.kLongCANTimeoutMs);
            System.out.println("zeroing turret at " + sensorVal);
        }
    }

    @Override
    public void stop() {
        camera.setEnabled(false);
    }

    // other
    @Override
    public boolean checkSystem() {
        boolean passed;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, .2);
        Timer.delay(2);
        var ticks = getActualTurretPositionTicks();
        var diff = Math.abs(ticks - TURRET_LIMIT_FORWARD);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualTurretPositionTicks();
        diff = Math.abs(ticks - TURRET_LIMIT_REVERSE);
        System.out.println(" - TICKS: " + ticks + "  ERROR: " + diff);
        passed = passed & diff <= 50;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        return passed;
    }

    public enum ControlMode {
        FIELD_FOLLOWING,
        CAMERA_FOLLOWING,
        POSITION,
        MANUAL,
    }
}
