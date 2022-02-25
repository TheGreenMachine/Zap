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

@Singleton
public class Turret extends Subsystem implements PidProvider {

    public static final double TURRET_JOG_SPEED = 0.25;
    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_EAST = 90; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_WEST = 270; // deg
    public static final String NAME = "turret";
    public static final int TURRET_LIMIT_REVERSE =
        ((int) factory.getConstant(NAME, "revLimit"));
    public static final int TURRET_LIMIT_FORWARD =
        ((int) factory.getConstant(NAME, "fwdLimit"));
    public final int ABS_TICKS_SOUTH;

    // Constants
    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 0;
    private final int TURRET_ENCODER_PPR;
    private final int TURRET_PPR;
    private final int TURRET_ENCODER_MASK;
    private final double TURRET_ENC_RATIO;
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static Turret INSTANCE;

    // Components
    private final IMotorControllerEnhanced turret;

    @Inject
    private static Camera camera;

    @Inject
    private static RobotState robotState;

    @Inject
    private static LedManager led;

    private final String pidSlot = "slot0";
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    // State
    private int desiredTurretPos = 0;
    private int followingTurretPos = 0;
    private double turretSpeed;
    private boolean outputsChanged;
    private ControlMode controlMode = ControlMode.MANUAL;

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");
        TURRET_ENCODER_PPR = (int) factory.getConstant(NAME, "encPPR");
        TURRET_PPR = (int) factory.getConstant(NAME, "turretPPR");
        TURRET_ENCODER_MASK = TURRET_PPR - 1;
        TURRET_ENC_RATIO = (double) TURRET_PPR / TURRET_ENCODER_PPR;
        ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));

        turret.setNeutralMode(NeutralMode.Brake);

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
        }
    }

    /**
     * converts 0-360 to 0-TURRET_ENCODER_PPR with zero  offset
     */
    public int convertTurretDegreesToTicks(double degrees) {
        // CCW is positive
        return (
            (int) ((((degrees) / 360.0) * TURRET_PPR) + ABS_TICKS_SOUTH) &
            TURRET_ENCODER_MASK
        );
    }

    /**
     * converts 0-TURRET_ENCODER_PPR with zero offset
     */
    public double convertTurretTicksToDegrees(double ticks) {
        double adjTicks = ((int) ticks - ABS_TICKS_SOUTH) & TURRET_ENCODER_MASK;
        return adjTicks / TURRET_PPR * 360;
    }

    @Override
    public synchronized void zeroSensors() {
        if (RobotBase.isSimulation()) {
            // populate sensor with offset
            turret.setSelectedSensorPosition(ABS_TICKS_SOUTH, 0, 0);
        }
        if (turret instanceof TalonSRX) {
            var sensors = ((TalonSRX) turret).getSensorCollection(); // not absolute
            // If we have a sensorVal < turret ppr then it is safe to reset
            if(sensors.getQuadraturePosition() < TURRET_PPR) { // ABSOLUTE
                //get absolute sensor value
                var sensorVal = sensors.getPulseWidthPosition();
                sensors.setQuadraturePosition(sensorVal, Constants.kLongCANTimeoutMs);
                System.out.println("zeroing turret at " + sensorVal);
            }
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
                turret.selectProfileSlot(kPIDGyroIDx, 0); // what are profile slots? - ginget
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

    public void setTurretSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        if (turretSpeed != speed) {
            turretSpeed = speed;
            outputsChanged = true;
        }
    }

    private synchronized void setTurretPosition(double position) {
        //Since we are using position we need ensure value stays in one rotation
        int adjPos = (int) position & TURRET_ENCODER_MASK;
        if (desiredTurretPos != adjPos) {
            desiredTurretPos = adjPos;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretPosition(convertTurretDegreesToTicks(angle));
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualTurretPositionDegrees());
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
    public void readFromHardware() {
        robotState.vehicle_to_turret =
            Rotation2d.fromDegrees(getActualTurretPositionDegrees());
        // show turret
        var robotPose = robotState.field.getRobotPose();
        var turret = robotState.field.getObject("turret");
        turret.setPose(
            robotPose.getX() - .1,
            robotPose.getY() + .1,
            Rotation2d.fromDegrees(robotState.getLatestFieldToTurret())
        );
    }

    @Override
    public void writeToHardware() {
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

    private void autoHome() {
        var angle = camera.getDeltaXAngle();
        int adj =
            convertTurretDegreesToTicks(angle * .10) +
            followingTurretPos -
            ABS_TICKS_SOUTH;
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void trackGyro() {
        // since convertTurretDegreesToTicks adjusts to zero we need to remove offset
        int fieldTickOffset =
            convertTurretDegreesToTicks(
                robotState.field_to_vehicle.getRotation().getDegrees()
            ) -
            ABS_TICKS_SOUTH;
        int adj = (desiredTurretPos + fieldTickOffset) & TURRET_ENCODER_MASK;
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl(double rawPos) {
        if (outputsChanged) {
            System.out.println(rawPos + " +++++");
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, rawPos);
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

    @Override
    public void stop() {
        camera.setEnabled(false);
    }

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
