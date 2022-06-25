package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
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

    // Constants
    public static final String NAME = "turret";
    public static final double JOG_SPEED = 0.15;
    public static final double SOUTH = 0; // deg - relative to vehicle NOT FIELD
    public static final double EAST = 270; // deg - relative to vehicle NOT FIELD
    public static final double NORTH = 180; // deg - relative to vehicle NOT FIELD
    public static final double WEST = 90; // deg - relative to vehicle NOT FIELD
    public final int REV_LIMIT = Math.min(
        (int) factory.getConstant(NAME, "fwdLimit"),
        ((int) factory.getConstant(NAME, "revLimit"))
    );
    public final int FWD_LIMIT = Math.max(
        (int) factory.getConstant(NAME, "fwdLimit"),
        ((int) factory.getConstant(NAME, "revLimit"))
    );
    public static int REV_WRAPAROUND_POINT; // lowest allowed tick value before turret masks (+turretPPR)
    public static int FWD_WRAPAROUND_POINT; // highest allowed tick value before turret masks (-turretPPR)
    public static int ZERO_OFFSET; // used to make sure turret tick range is non-negative
    public final int ABS_TICKS_SOUTH; // abs encoder count at cardinal SOUTH
    public static int ABS_PPR;
    public static int TURRET_PPR;
    private final double ENC_RATIO;
    public final double DELTA_X_SCALAR;

    private static final int kPrimaryCloseLoop = 0;
    private static final int kPIDGyroIDx = 0;
    private static final int kPIDVisionIDx = 0;
    private final PIDSlotConfiguration pidConfig;

    // Components
    private final IGreenMotor turret;

    @Inject
    private static Camera camera;

    @Inject
    private static LedManager led;

    // State
    private int desiredPos = 0;
    private int followingPos = 0;
    private int visionCorroboration = 0;
    private double turretSpeed;
    private boolean outputsChanged = true;
    private ControlMode controlMode;
    private final AsyncTimer waitForSnap = new AsyncTimer(
        .25,
        () -> {
            followingPos =
                (int) (getActualTurretPositionTicks() + cameraFollowingOffset());
            System.out.println("done aiming!");
            outputsChanged = true;
        }
    );

    public Turret() {
        super(NAME);
        this.turret = factory.getMotor(NAME, "turret");
        DELTA_X_SCALAR = factory.getConstant(NAME, "deltaXScalar", 1);

        ZERO_OFFSET = (int) factory.getConstant(NAME, "zeroOffset"); //add offset to keep turret in positive range
        ABS_TICKS_SOUTH = ((int) factory.getConstant(NAME, "absPosTicksSouth"));
        ABS_PPR = (int) factory.getConstant(NAME, "encPPR");
        TURRET_PPR = (int) factory.getConstant(NAME, "turretPPR");
        ENC_RATIO = (double) TURRET_PPR / ABS_PPR;

        int MASK = 0;
        if (Math.abs(REV_LIMIT - FWD_LIMIT) > TURRET_PPR) {
            MASK = Math.abs((REV_LIMIT + TURRET_PPR) - (FWD_LIMIT)) / 2; // this value is truncated
        }
        FWD_WRAPAROUND_POINT = FWD_LIMIT + MASK;
        REV_WRAPAROUND_POINT = REV_LIMIT - MASK;

        turret.setNeutralMode(NeutralMode.Brake);

        // Position Control
        double peakOutput = 0.75;
        pidConfig = factory.getPidSlotConfig(NAME);
        turret.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        turret.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        turret.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        turret.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
        turret.configAllowableClosedloopError(
            kPIDGyroIDx,
            pidConfig.allowableError.intValue(),
            Constants.kCANTimeoutMs
        );
        turret.configAllowableClosedloopError(
            kPIDVisionIDx,
            pidConfig.allowableError.intValue(),
            Constants.kCANTimeoutMs
        );

        // Soft Limits
        turret.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turret.configForwardSoftLimitThreshold(FWD_LIMIT, Constants.kCANTimeoutMs); // Forward = MAX
        turret.configReverseSoftLimitThreshold(REV_LIMIT, Constants.kCANTimeoutMs); // Reverse = MIN
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
            int halfAbsPPR = ABS_PPR / 2;
            int absSensorVal = sensors.getPulseWidthPosition(); // absolute
            int offset = ZERO_OFFSET - absSensorVal + halfAbsPPR;

            // It is safe to reset quadrature if turret enc reads ~0 (on startup)
            if (
                Math.abs(sensors.getQuadraturePosition()) < halfAbsPPR ||
                (int) ENC_RATIO == 1
            ) {
                //second check - don't zero if abs enc not in viable range
                if (absSensorVal > -1 && absSensorVal < ABS_PPR) {
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
        if (desiredPos != (int) position) {
            desiredPos = (int) position;
            outputsChanged = true;
        }
    }

    // CCW positive - 0 to 360
    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        System.out.println("setting turret angle: " + angle);
        setTurretPosition(convertTurretDegreesToTicks(angle));
        followingPos = desiredPos;
    }

    public synchronized void snapWithCamera() {
        waitForSnap.reset();
        setControlMode(ControlMode.CAMERA_SNAP);
    }

    public synchronized void setFollowingAngle(double angle) {
        if (angle < 0) {
            angle += 360; // if angle is negative, wrap around - we only deal with values from 0 to 360
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
            return desiredPos;
        }
        return followingPos;
    }

    public double getPositionError() {
        return getTargetPosition() - getActualTurretPositionTicks();
    }

    @Override
    public void readFromHardware() {
        if (followingPos > 2 * TURRET_PPR) {
            followingPos %= TURRET_PPR;
        }

        robotState.vehicleToTurret =
            Rotation2d.fromDegrees(getActualTurretPositionDegrees());
    }

    @Override
    public void writeToHardware() {
        switch (controlMode) {
            case CAMERA_FOLLOWING:
                autoHome();
                positionControl(followingPos);
                break;
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl(followingPos);
                break;
            case CENTER_FOLLOWING:
                trackCenter();
                positionControl(followingPos);
                break;
            case ABSOLUTE_FOLLOWING:
                trackAbsolute();
                positionControl(followingPos);
                break;
            case ABSOLUTE_MADNESS:
                autoHomeWithOffset(motionOffset());
                positionControl(followingPos);
                break;
            case CAMERA_SNAP:
                snapControl();
                positionControl(followingPos);
            case POSITION:
                positionControl(desiredPos);
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    private int cameraFollowingOffset() {
        var delta = -camera.getDeltaX();
        return ((int) (delta * DELTA_X_SCALAR));
    }

    private int fieldFollowingOffset() {
        return -convertTurretDegreesToTicks( // currently negated because motor is running counterclockwise
            robotState.fieldToVehicle.getRotation().getDegrees()
        );
    }

    private int centerFollowingOffset() {
        double opposite =
            Constants.fieldCenterY - robotState.getFieldToTurretPos().getY();
        double adjacent =
            Constants.fieldCenterX - robotState.getFieldToTurretPos().getX();
        double turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    private int motionOffset() {
        Translation2d shooterAxis = new Translation2d(
            robotState.shooterMPS,
            robotState.getLatestFieldToTurret()
        );
        Translation2d driveAxis = new Translation2d(
            robotState.deltaVehicle.vxMetersPerSecond,
            robotState.deltaVehicle.vyMetersPerSecond
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
        int adj = followingPos + cameraOffset;
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void autoHomeWithOffset(int offset) {
        var cameraOffset = cameraFollowingOffset();
        int adj = followingPos + cameraOffset + offset;
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    public void snapControl() {
        waitForSnap.update();
    }

    private void trackGyro() {
        int fieldTickOffset = fieldFollowingOffset();
        int adj = (desiredPos + fieldTickOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void trackCenter() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();

        int adj = (desiredPos + fieldTickOffset + centerOffset + visionCorroboration);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void trackAbsolute() {
        int fieldTickOffset = fieldFollowingOffset();
        int centerOffset = centerFollowingOffset();
        int motionOffset = motionOffset();

        int adj = (desiredPos + fieldTickOffset + centerOffset + motionOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void positionControl(int rawPos) {
        if (outputsChanged) {
            if (rawPos > FWD_WRAPAROUND_POINT) {
                rawPos -= TURRET_PPR;
            } else if (rawPos < REV_WRAPAROUND_POINT) {
                rawPos += TURRET_PPR;
            }
            int adjPos = (rawPos + ABS_TICKS_SOUTH + ZERO_OFFSET);

            turret.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, adjPos);
            outputsChanged = false;
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            turret.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                turretSpeed
            );
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
        var diff = Math.abs(ticks - FWD_LIMIT);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        turret.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualTurretPositionTicks();
        diff = Math.abs(ticks - REV_LIMIT);
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
