package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.math.PoseUtil;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

@Singleton
public class Turret extends Subsystem implements PidProvider {

    // Constants
    public static final String NAME = "turret";
    public static final double kJogSpeed = 0.5;
    public static final double kSouth = 0; // deg - relative to vehicle NOT FIELD
    public static final double kEast = 270; // deg - relative to vehicle NOT FIELD
    public static final double kNorth = 180; // deg - relative to vehicle NOT FIELD
    public static final double kWest = 90; // deg - relative to vehicle NOT FIELD
    public final int kRevLimit;
    public final int kFwdLimit;
    public static int kRevWrapAroundPos; // lowest allowed tick value before turret masks (+turretPPR)
    public static int kFwdWrapAroundPos; // highest allowed tick value before turret masks (-turretPPR)
    public final int kAbsTicksSouthOffset; // abs encoder count at cardinal SOUTH
    public static int kAbsPPR;
    public static int kTurretPPR;
    private final double kRatioTurretAbs;
    public final double kDeltaXScalar;

    private final AsyncTimer snapTimer = new AsyncTimer(
        .25,
        () -> {
            followingPos = (int) (getActualPosTicks() + cameraFollowingOffset());
            System.out.println("done aiming!");
            outputsChanged = true;
        }
    );

    private static final int kPrimaryCloseLoop = 0;
    private final PIDSlotConfiguration pidConfig;

    // Components
    private final IGreenMotor turretMotor;

    private static Camera camera;

    private static LedManager led;

    // State
    private int desiredPos = 0;
    private int followingPos = 0;
    private boolean lostEncPos = false;
    private int visionCorroboration = 0;
    private double turretSpeed;
    private boolean outputsChanged = true;
    private ControlMode controlMode;

    @Inject
    public Turret(
        Camera camera,
        LedManager ledManager,
        Infrastructure inf,
        RobotState rs
    ) {
        super(NAME, inf, rs);
        this.camera = camera;
        led = ledManager;
        turretMotor = factory.getMotor(NAME, "turretMotor");
        kDeltaXScalar = factory.getConstant(NAME, "deltaXScalar", 1);

        // Define PPR values and determine whether to offset set positions by absEnc south pos
        kAbsPPR = (int) factory.getConstant(NAME, "absPPR");
        kTurretPPR = (int) factory.getConstant(NAME, "turretPPR");
        kRatioTurretAbs = (double) kTurretPPR / kAbsPPR;
        kAbsTicksSouthOffset =
            kRatioTurretAbs == 1
                ? ((int) factory.getConstant(NAME, "absPosTicksSouth"))
                : 0;

        // define limits + when turret should wrap around
        kRevLimit = ((int) factory.getConstant(NAME, "revLimit"));
        kFwdLimit = (int) factory.getConstant(NAME, "fwdLimit");
        int MASK = 0;
        if (Math.abs(kRevLimit - kFwdLimit) > kTurretPPR) {
            MASK = Math.abs((kRevLimit + kTurretPPR) - (kFwdLimit)) / 2; // this value is truncated
        }
        kFwdWrapAroundPos = kFwdLimit + MASK;
        kRevWrapAroundPos = kRevLimit - MASK;

        // Position Control
        double peakOutput = 0.5;
        pidConfig = factory.getPidSlotConfig(NAME);
        turretMotor.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        turretMotor.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        turretMotor.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        turretMotor.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);

        // Soft Limits
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turretMotor.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turretMotor.configForwardSoftLimitThreshold(kFwdLimit, Constants.kCANTimeoutMs); // Forward = MAX
        turretMotor.configReverseSoftLimitThreshold(kRevLimit, Constants.kCANTimeoutMs); // Reverse = MIN
        turretMotor.overrideLimitSwitchesEnable(true);
        turretMotor.overrideSoftLimitsEnable(true);
    }

    /**
     * converts 0-360 to 0-TURRET_ENCODER_PPR
     */
    public static int convertTurretDegreesToTicks(double degrees) {
        return (int) (degrees / 360.0 * kTurretPPR);
    }

    /**
     * converts 0-TURRET_ENCODER_PPR
     */
    public static double convertTurretTicksToDegrees(double ticks) {
        return ticks / kTurretPPR * 360;
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(false);
    }

    public synchronized void zeroSensors(boolean resetEncPos) {
        desiredPos = 0;
        followingPos = 0;
        lostEncPos = false;

        if ((int) kRatioTurretAbs == 1) {
            var sensorVal =
                ((IMotorSensor) turretMotor).getPulseWidthPosition() % kAbsPPR;
            ((IMotorSensor) turretMotor).setQuadraturePosition(sensorVal);
            System.out.println("zeroing turret at " + sensorVal);
        } else {
            if (resetEncPos) {
                turretMotor.setSelectedSensorPosition(
                    0,
                    kPrimaryCloseLoop,
                    Constants.kCANTimeoutMs
                );
            }
        }
    }

    public ControlMode getControlMode() {
        return controlMode;
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            outputsChanged = true;
            this.controlMode = controlMode;
            System.out.println("turret controlMode: " + this.controlMode);
        }
    }

    public void setTurretSpeed(double speed) {
        setControlMode(ControlMode.MANUAL);
        if (turretSpeed != speed) {
            turretSpeed = speed;
            outputsChanged = true;
        }
    }

    private synchronized void setDesiredPos(double position) {
        if (desiredPos != (int) position) {
            desiredPos = (int) position;
            outputsChanged = true;
        }
    }

    // CCW positive - 0 to 360
    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        System.out.println("setting turret angle: " + angle);
        setDesiredPos(convertTurretDegreesToTicks(angle));
        followingPos = desiredPos;
    }

    public synchronized void setFollowingAngle(double angle) {
        setDesiredPos(convertTurretDegreesToTicks(angle));
    }

    public synchronized void snapWithCamera() {
        snapTimer.reset();
        setControlMode(ControlMode.CAMERA_SNAP);
    }

    public synchronized void lockTurret() {
        setTurretAngle(getActualPosDegrees());
    }

    public double getActualPosDegrees() {
        return convertTurretTicksToDegrees(getActualPosTicks());
    }

    // this is what is eventually referred to in readFromHardware, so we're undoing conversions here
    public double getActualPosTicks() {
        return (
            (
                turretMotor.getSelectedSensorPosition(kPrimaryCloseLoop) -
                kAbsTicksSouthOffset
            )
        );
    }

    public double getDesiredPosTicks() {
        if (controlMode == ControlMode.POSITION) {
            return desiredPos;
        }
        return followingPos;
    }

    public double getPosError() {
        return getDesiredPosTicks() - getActualPosTicks();
    }

    @Override
    public void readFromHardware() {
        if (followingPos > 2 * kTurretPPR) {
            followingPos %= kTurretPPR;
        }

        if (turretMotor.hasResetOccurred()) {
            System.out.println("turretMotor lost its position!");
            led.setDefaultStatus(LedManager.RobotStatus.ERROR);
            lostEncPos = true;
        }

        robotState.vehicleToTurret = Rotation2d.fromDegrees(getActualPosDegrees());
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
                break;
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
        return ((int) (delta * kDeltaXScalar));
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
        snapTimer.update();
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

    private void positionControl(int pos) {
        if (outputsChanged) {
            outputsChanged = false;
            if (lostEncPos) {
                manualControl();
                return;
            }

            if (pos > kFwdWrapAroundPos) {
                pos -= kTurretPPR;
            } else if (pos < kRevWrapAroundPos) {
                pos += kTurretPPR;
            }
            int rawPos = (pos + kAbsTicksSouthOffset);

            turretMotor.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                rawPos
            );
        }
    }

    private void manualControl() {
        if (outputsChanged) {
            turretMotor.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                turretSpeed
            );
            outputsChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        return pidConfig;
    }

    @Override
    public boolean checkSystem() {
        boolean passed;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, .2);
        Timer.delay(2);
        var ticks = getActualPosTicks();
        var diff = Math.abs(ticks - kFwdLimit);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualPosTicks();
        diff = Math.abs(ticks - kRevLimit);
        System.out.println(" - TICKS: " + ticks + "  ERROR: " + diff);
        passed = passed & diff <= 50;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
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
