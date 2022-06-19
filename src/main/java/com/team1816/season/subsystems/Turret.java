package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.math.PoseUtil;
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

    public static final double TURRET_JOG_SPEED = 0.15;
    public static final double CARDINAL_SOUTH = 0; // deg
    public static final double CARDINAL_EAST = 270; // deg
    public static final double CARDINAL_NORTH = 180; // deg
    public static final double CARDINAL_WEST = 90; // deg
    public static final String NAME = "turret";
    private static int HALF_ABS_ENCPPR;
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
    public final double DELTA_X_SCALAR;
    public static int TURRET_ABS_ENCODER_PPR = 4096;
    public static int TRUE_TURRET_PPR; // (TUR) new edits
    public static int TURRET_PPR; // (TUR) new edits
    private final int TURRET_MASK;
    private final int TRUE_TURRET_MASK;
    private final double TURRET_ENC_RATIO;
    public final int ALLOWABLE_ERROR_TICKS;

    // Components
    private final IMotorControllerEnhanced turret;

    @Inject
    private static Camera camera;

    @Inject
    private static LedManager led;

    private final String pidSlot = "slot0";
    private final PIDSlotConfiguration pidConfig;
    // State
    private int desiredTurretPos = 0;
    private int followingTurretPos = 0;
    private int visionCorroboration = 0;
    private double turretSpeed;
    private boolean outputsChanged = true;
    private ControlMode controlMode;
    private AsyncTimer waitForSnap = new AsyncTimer(
        .25,
        () -> {
            followingTurretPos =
                (int) (getActualTurretPositionTicks() + cameraFollowingOffset());
            System.out.println("done aiming!");
            outputsChanged = true;
        }
    );

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");
        DELTA_X_SCALAR = factory.getConstant(NAME, "deltaXScalar", 1);
        TURRET_ABS_ENCODER_PPR = (int) factory.getConstant(NAME, "encPPR");
        TRUE_TURRET_PPR = (int) factory.getConstant(NAME, "turretPPR");
        TRUE_TURRET_MASK = TRUE_TURRET_PPR - 1;
        TURRET_ENC_RATIO = (double) TRUE_TURRET_PPR / TURRET_ABS_ENCODER_PPR;
        ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));
        HALF_ABS_ENCPPR = TURRET_ABS_ENCODER_PPR / 2 - HALF_ABS_ENCPPR;
        ZERO_OFFSET = (int) factory.getConstant(NAME, "zeroOffset"); //add offset to keep turret in positive range
        turret.setNeutralMode(NeutralMode.Brake);

        //there's absolutely no reason to have more than two rotations
        TURRET_PPR =
            TRUE_TURRET_PPR *
            (
                Math.abs(TURRET_LIMIT_FORWARD - TURRET_LIMIT_REVERSE) /
                    TRUE_TURRET_PPR >
                    1
                    ? 2
                    : 1
            ); //TUR
        TURRET_MASK = TURRET_PPR - 1;

        pidConfig = factory.getPidSlotConfig(NAME, pidSlot);
        ALLOWABLE_ERROR_TICKS = pidConfig.allowableError.intValue();
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

    /**
     * converts 0-360 to 0-TURRET_ENCODER_PPR with zero offset
     */
    public static int convertTurretDegreesToTicks(double degrees) {
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
            var absSensorVal = sensors.getPulseWidthPosition(); // absolute
            var offset = ZERO_OFFSET - absSensorVal + HALF_ABS_ENCPPR;

            // It is safe to reset quadrature if turret enc reads ~0 (on startup)
            if (
                Math.abs(sensors.getQuadraturePosition()) < HALF_ABS_ENCPPR ||
                (int) TURRET_ENC_RATIO == 1
            ) {
                //second check - don't zero if abs enc not in viable range
                if (absSensorVal > -1 && absSensorVal < TURRET_ABS_ENCODER_PPR) {
                    sensors.setQuadraturePosition(offset);
                    System.out.println("Zeroing turret limits! Offset: " + offset);
                } else {
                    DriverStation.reportError(
                        "ABSOLUTE ENCODER INVALID RANGE - not zeroing",
                        false
                    );
                }
            } else {
                System.out.println("UNSAFE - NOT ZEROING TURRET QUADRATURE");
            }
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            outputsChanged = true;
            if (
                controlMode == ControlMode.CAMERA_FOLLOWING ||
                controlMode == ControlMode.CAMERA_SNAP
            ) {
                if (Constants.kUseVision) {
                    this.controlMode = controlMode;
                    turret.selectProfileSlot(kPIDVisionIDx, 0);
                    camera.setCameraEnabled(true);
                    led.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
                } else {
                    System.out.println("auto aim not enabled! Not aiming with camera!");
                }
            } else {
                this.controlMode = controlMode;
                turret.selectProfileSlot(kPIDGyroIDx, 0);
                camera.setCameraEnabled(controlMode == ControlMode.MANUAL);
                if (controlMode == ControlMode.MANUAL) {
                    led.indicateStatus(LedManager.RobotStatus.MANUAL_TURRET);
                } else {
                    led.indicateDefaultStatus();
                }
            }
            System.out.println("TURRET CONTROL MODE IS . . . . . . " + this.controlMode);
        }
    }

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        return pidConfig;
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
            //            System.out.println("setting desiredTurretPos to " + position);
            desiredTurretPos = (int) position;
            outputsChanged = true;
        }
    }

    // CCW positive - 0 to 360
    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        System.out.println("setting turret angle: " + angle);
        setTurretPosition(convertTurretDegreesToTicks(angle));
        followingTurretPos = desiredTurretPos;
    }

    public synchronized void snapWithCamera() {
        waitForSnap.reset();
        setControlMode(ControlMode.CAMERA_SNAP);
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

    // this is what is eventually referred to in readFromHardware, so we're undoing conversions here
    public double getActualTurretPositionTicks() {
        return (
            (
                turret.getSelectedSensorPosition(kPrimaryCloseLoop) -
                ABS_TICKS_SOUTH -
                ZERO_OFFSET
            )
        );
    }

    public double getTargetPosition() {
        if (controlMode == ControlMode.POSITION) {
            return desiredTurretPos;
        }
        return followingTurretPos;
    }

    public double getPositionError() {
        return getTargetPosition() - getActualTurretPositionTicks();
    }

    @Override
    public void readFromHardware() {
        desiredTurretPos %= TURRET_PPR;
        followingTurretPos %= TURRET_PPR;

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
                positionControl(followingTurretPos);
                break;
            case ABSOLUTE_FOLLOWING:
                trackAbsolute();
                positionControl(followingTurretPos);
                break;
            case ABSOLUTE_MADNESS:
                autoHomeWithOffset(motionOffset());
                positionControl(followingTurretPos);
                break;
            case CAMERA_SNAP:
                snapControl();
                positionControl(followingTurretPos);
            case POSITION:
                positionControl(desiredTurretPos);
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    private int cameraFollowingOffset() {
        var delta = -camera.getDeltaX();
        return ((int) (delta * DELTA_X_SCALAR)) - ABS_TICKS_SOUTH;
    }

    private int fieldFollowingOffset() {
        return -convertTurretDegreesToTicks( // currently negated because motor is running counterclockwise
            robotState.field_to_vehicle.getRotation().getDegrees()
        );
    }

    private int centerFollowingOffset() {
        double opposite =
            Constants.fieldCenterY - robotState.getFieldToTurretPos().getY();
        double adjacent =
            Constants.fieldCenterX - robotState.getFieldToTurretPos().getX();
        double turretAngle = 0;
        turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    private int motionOffset() {
        Translation2d shooterAxis = new Translation2d(
            robotState.shooterSpeed,
            robotState.getLatestFieldToTurret()
        );
        Translation2d driveAxis = new Translation2d(
            robotState.delta_vehicle.vxMetersPerSecond,
            robotState.delta_vehicle.vyMetersPerSecond
        );
        Translation2d predictedTrajectory = driveAxis.unaryMinus().plus(shooterAxis);
        double motionOffsetAngle = PoseUtil.getAngleBetween(
            predictedTrajectory,
            shooterAxis
        );

        if (motionOffsetAngle > Math.PI) {
            motionOffsetAngle -= Math.PI * 2;
        }
        return convertTurretDegreesToTicks(Units.radiansToDegrees(motionOffsetAngle));
    }

    private void autoHome() {
        var cameraOffset = cameraFollowingOffset();
        int adj = followingTurretPos + cameraOffset;
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    private void autoHomeWithOffset(int offset) {
        var cameraOffset = cameraFollowingOffset();
        int adj = followingTurretPos + cameraOffset + (int) (offset * DELTA_X_SCALAR);
        if (adj != followingTurretPos) {
            followingTurretPos = adj;
            outputsChanged = true;
        }
    }

    public void snapControl() {
        waitForSnap.update();
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

        int adj =
            (desiredTurretPos + fieldTickOffset + centerOffset + visionCorroboration);
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
        int adjPos = (rawPos + ABS_TICKS_SOUTH + ZERO_OFFSET) % TURRET_MASK; // % TRUE_TURRET_MASK; (TUR)
        // we want to mod it by the TRUE_TURRET_MASK only if it's not in the fringes
        /**
         * TUR CHANGES BELOW
         */
        if (!(Math.abs(TURRET_LIMIT_FORWARD - TURRET_LIMIT_REVERSE) > TRUE_TURRET_PPR)) { // if this is false then TURRET_MASK = TRUE_TURRET_MASK
            if (
                !(
                    (
                        adjPos <
                        (
                            Math.min(TURRET_LIMIT_FORWARD, TURRET_LIMIT_REVERSE) +
                            (
                                Math.abs(TURRET_LIMIT_REVERSE - TURRET_LIMIT_FORWARD) -
                                TRUE_TURRET_PPR
                            ) /
                            2
                        ) &&
                        adjPos > (Math.min(TURRET_LIMIT_FORWARD, TURRET_LIMIT_REVERSE))
                    ) ||
                    (
                        adjPos >
                        (
                            Math.max(TURRET_LIMIT_FORWARD, TURRET_LIMIT_REVERSE) -
                            (
                                Math.abs(TURRET_LIMIT_REVERSE - TURRET_LIMIT_FORWARD) -
                                TRUE_TURRET_PPR
                            ) /
                            2
                        ) &&
                        adjPos < (Math.max(TURRET_LIMIT_FORWARD, TURRET_LIMIT_REVERSE))
                    ) // everything above should address the fringes
                )
            ) {
                adjPos %= TRUE_TURRET_MASK;
            }
        }
        /**
         * TUR CHANGES ABOVE
         */
        if (adjPos < 0) {
            adjPos += TRUE_TURRET_PPR; //Shouldn't this be turret ppr?
        }
        if (outputsChanged) {
            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, adjPos);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            //            if (turretSpeed == 0) {
            //                turret.set(
            //                    com.ctre.phoenix.motorcontrol.ControlMode.Position,
            //                    getActualTurretPositionTicks() + 200 * turret.getMotorOutputPercent()
            //                );
            //            } else {
            turret.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                turretSpeed
            );
            //            }
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

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
        ABSOLUTE_FOLLOWING,
        ABSOLUTE_MADNESS,
        CAMERA_SNAP,
        POSITION,
        MANUAL,
    }
}
