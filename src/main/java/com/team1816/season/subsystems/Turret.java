package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Turret extends Subsystem implements PidProvider {

    public static final double TURRET_JOG_SPEED = 0.04;
    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_EAST = 270; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final String NAME = "turret";
    private static int HALF_ENCPPR;
    public static int TURRET_LIMIT_REVERSE =
        ((int) factory.getConstant(NAME, "revLimit"));
    public static int TURRET_LIMIT_FORWARD =
        ((int) factory.getConstant(NAME, "fwdLimit"));
    public final int ABS_TICKS_SOUTH;
    public static int ZERO_OFFSET; //used to make sure turret tick range is non-negative

    // Constants
    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 0;
    public static int TURRET_ENCODER_PPR = 4096;
    public final int TURRET_PPR;
    private final int TURRET_MASK;
    private final double TURRET_ENC_RATIO;
    private static final int ALLOWABLE_ERROR_TICKS = 5;
    private static Turret INSTANCE;

    // Components
    private final IMotorControllerEnhanced turret;

    @Inject
    private static Camera camera;

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
    private boolean outputsChanged = true;
    private ControlMode controlMode = ControlMode.MANUAL;

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");
        TURRET_ENCODER_PPR = (int) factory.getConstant(NAME, "encPPR");
        TURRET_PPR = (int) factory.getConstant(NAME, "turretPPR");
        TURRET_MASK = TURRET_PPR - 1;
        TURRET_ENC_RATIO = (double) TURRET_PPR / TURRET_ENCODER_PPR;
        ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));
        HALF_ENCPPR = TURRET_ENCODER_PPR / 2 - HALF_ENCPPR;
        ZERO_OFFSET = (int) factory.getConstant(NAME, "zeroOffset") + HALF_ENCPPR; //add offset to keep turret in positive range
        turret.setNeutralMode(NeutralMode.Brake);

        PIDSlotConfiguration pidConfig = factory.getPidSlotConfig(NAME, pidSlot);
        this.kP = pidConfig.kP;
        this.kI = pidConfig.kI;
        this.kD = pidConfig.kD;
        this.kF = pidConfig.kF;
        synchronized (this) {
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
     * converts 0-360 to 0-TURRET_ENCODER_PPR with zero offset
     */
    public int convertTurretDegreesToTicks(double degrees) {
        return ((int) (((degrees) / 360.0) * TURRET_PPR));
    }

    /**
     * converts 0-TURRET_ENCODER_PPR with zero offset
     */
    public double convertTurretTicksToDegrees(double ticks) {
        return ticks / TURRET_PPR * 360;
    }

    @Override
    public synchronized void zeroSensors() {
        if (turret instanceof IMotorSensor) {
            var sensors = (IMotorSensor) turret;
            // If we have a sensor position that resides within the same absolute encoder revolution as the abs_ticks_south position then it is safe to reset
            if (
                sensors.getQuadraturePosition() < TURRET_ENCODER_PPR &&
                sensors.getQuadraturePosition() > -1
            ) {
                //get absolute sensor value
                var sensorVal = sensors.getPulseWidthPosition(); // absolute
                if (sensorVal > -1 && sensorVal < TURRET_ENCODER_PPR) {
                    var offset = ZERO_OFFSET - sensorVal + HALF_ENCPPR;
                    sensors.setQuadraturePosition(offset);
                    System.out.println("Zeroing turret! Offset: " + offset);
                } else {
                    DriverStation.reportError(
                        "ABSOLUTE ENCODER INVALID RANGE - not zeroing",
                        false
                    );
                }
            } else {
                System.out.println("unsafe to zero turret sensors! NOT ZEROING");
                // should we directly make turret into manual control at this point?
            }
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            if (controlMode == ControlMode.CAMERA_FOLLOWING) {
                if (Constants.kUseVision) {
                    turret.selectProfileSlot(kPIDVisionIDx, 0);
                    this.controlMode = controlMode;
                    camera.setEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                } else {
                    System.out.println("auto aim not enabled! Not aiming with camera!");
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
            System.out.println("TURRET CONTROL MODE IS " + controlMode);
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
        if (desiredTurretPos != (int) position) {
            desiredTurretPos = (int) position;
            outputsChanged = true;
        }
    }

    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        setTurretPosition(convertTurretDegreesToTicks(angle));
    }

    public synchronized void setFollowingAngle(double angle) {
        if (angle < 0) {
            angle = angle + 360; // if angle is negative, wrap around - we only deal with values from 0 to 360
        }
        setTurretPosition(convertTurretDegreesToTicks(angle));
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualTurretPositionDegrees());
    }

    public double getActualTurretPositionDegrees() {
        return convertTurretTicksToDegrees(getActualTurretPositionTicks());
    }

    // this is what is eventually referred to in readFromHardware so we're undoing conversions here
    public double getActualTurretPositionTicks() {
        return (
            (
                turret.getSelectedSensorPosition(kPrimaryCloseLoop) -
                ABS_TICKS_SOUTH -
                ZERO_OFFSET
            ) %
            TURRET_MASK
        );
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
            case CENTER_FOLLOWING:
                trackCenter();
                //autoHome();
                positionControl(followingTurretPos);
                break;
            case ABSOLUTE_MADNESS:
                trackAbsolute();
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

    private int fieldFollowingOffset() {
        return -convertTurretDegreesToTicks( // currently negated because motor is running counterclockwise
            robotState.field_to_vehicle.getRotation().getDegrees()
        );
    }

    private int centerFollowingOffset() {
        double opposite = Constants.fieldCenterY - robotState.field_to_vehicle.getY();
        double adjacent = Constants.fieldCenterX - robotState.field_to_vehicle.getX();
        double turretAngle = 0;
        turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(
            Units.radiansToDegrees(turretAngle)
        );
    }

    private int motionOffset() {
        Translation2d shooterAxis = new Translation2d(
            robotState.getCurrentShooterSpeedMetersPerSecond(),
            Rotation2d.fromDegrees(robotState.getLatestFieldToTurret())
        );
        Translation2d driveAxis = new Translation2d(
            robotState.chassis_speeds.vxMetersPerSecond,
            robotState.chassis_speeds.vyMetersPerSecond
        );
        Translation2d predictedTrajectory = driveAxis.unaryMinus().plus(shooterAxis);
        double motionOffsetAngle = getAngleBetween(predictedTrajectory, shooterAxis);

        if (motionOffsetAngle > Math.PI) {
            motionOffsetAngle -= Math.PI * 2;
        }
        return convertTurretDegreesToTicks(
            Units.radiansToDegrees(motionOffsetAngle)
        );
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
        int fieldTickOffset = fieldFollowingOffset();
        int adj = (desiredTurretPos + fieldTickOffset);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void trackCenter() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();

        int adj = (desiredTurretPos + fieldTickOffset + centerOffset);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void trackAbsolute() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();
        int motionOffset = motionOffset();

        int adj = (desiredTurretPos + fieldTickOffset + centerOffset + motionOffset);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl(int rawPos) {
        int adjPos = (rawPos + ABS_TICKS_SOUTH + ZERO_OFFSET) % TURRET_MASK;
        if (outputsChanged) {
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, adjPos);
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

    private double getAngleBetween(Translation2d a, Translation2d b) {
        double dot = (a.getNorm() * b.getNorm() == 0)
            ? 0
            : Math.acos(
            (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm())
        );
        double cross = crossProduct(a, b);
        if(cross > 0) {
            dot*=-1;
        }
        return dot;
    }

    private static double crossProduct(Translation2d a, Translation2d b) {
        double [] vect_A = {a.getX(), a.getY(), 0};
        double [] vect_B = {b.getX(), b.getY(), 0};
        double cross_P = vect_A[0] * vect_B[1]
            - vect_A[1] * vect_B[0];
        return cross_P;
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
        CAMERA_FOLLOWING,
        FIELD_FOLLOWING,
        CENTER_FOLLOWING,
        ABSOLUTE_MADNESS,
        POSITION,
        MANUAL,
    }
}
