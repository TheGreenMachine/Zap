package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** This class models a multi-functional turret with target and field tracking abilities */
@Singleton
public class Turret extends Subsystem implements PidProvider {

    /** constants */
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

    private static final int kPrimaryCloseLoop = 0;
    private final PIDSlotConfiguration pidConfig;

    /** components */
    private final IGreenMotor turretMotor;

    private static Camera camera;

    private static LedManager led;

    /** state */
    private int desiredPos = 0;
    private int followingPos = 0;
    private boolean lostEncPos = false;
    private boolean deadzone = false;
    private int visionCorroboration = 0;
    private double turretSpeed; // this is used as a percent output demand, NOT THE SAME AS turretVelocity
    private double turretVelocity = 0d; // used to calculate acceleration and other gyroscopic effects
    private double turretRotationalAcceleration = 0d;
    private double turretCentripetalAcceleration = 0d;
    private boolean outputsChanged = true;
    private Pose2d target = Constants.targetPos;
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
        kRevLimit =
            Math.min(
                (int) factory.getConstant(NAME, "fwdLimit"),
                (int) factory.getConstant(NAME, "revLimit")
            );
        kFwdLimit =
            Math.max(
                (int) factory.getConstant(NAME, "fwdLimit"),
                (int) factory.getConstant(NAME, "revLimit")
            );
        int MASK = Math.abs((kRevLimit + kTurretPPR) - (kFwdLimit)) / 2; // this value is truncated
        kFwdWrapAroundPos = kFwdLimit + MASK;
        kRevWrapAroundPos = kRevLimit - MASK;

        // Position Control
        double peakOutput = 0.75;
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
            var sensor = ((IMotorSensor) turretMotor);
            var sensorVal = sensor.getPulseWidthPosition() % kAbsPPR;
            sensor.setQuadraturePosition(sensorVal);
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

    public void setTarget(Pose2d target) {
        if (this.target != target) {
            this.target = target;
        }
    }

    public synchronized void snap() {
        setControlMode(ControlMode.CENTER_FOLLOWING);
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

    /** periodic */
    @Override
    public void readFromHardware() {
        if (followingPos > 2 * kTurretPPR) {
            followingPos %= kTurretPPR;
        }

        deadzone =
            (followingPos >= kFwdWrapAroundPos || followingPos <= kRevWrapAroundPos);
        outputToSmartDashboard();

        double sensorVel = turretMotor.getSelectedSensorVelocity(0) / 10d;
        turretRotationalAcceleration =
            Units.degreesToRadians(
                convertTurretTicksToDegrees(sensorVel - turretVelocity) /
                Constants.kLooperDt
            );
        turretCentripetalAcceleration =
            Math.pow(Units.degreesToRadians(convertTurretTicksToDegrees(sensorVel)), 2) *
            Constants.kTurretZedRadius;
        turretVelocity = sensorVel;

        if (turretMotor.hasResetOccurred()) {
            System.out.println("turretMotor lost its position!");
            led.setDefaultStatus(LedManager.RobotStatus.ERROR);
            lostEncPos = true;
        }

        robotState.vehicleToTurret = Rotation2d.fromDegrees(getActualPosDegrees());
        robotState.fieldToTurret =
            new Pose2d(
                robotState.fieldToVehicle
                    .transformBy(
                        new Transform2d(
                            Constants.kTurretMountingOffset,
                            Constants.EmptyRotation
                        )
                    )
                    .getTranslation(),
                robotState.fieldToVehicle.getRotation().plus(robotState.vehicleToTurret)
            );
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
                setTarget(Constants.fieldCenterPose);
                trackTarget();
                positionControl(followingPos);
                break;
            case ABSOLUTE_FOLLOWING:
                trackAbsolute();
                positionControl(followingPos);
                break;
            case EJECT:
                eject();
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

    public void revolve() {
        switch (controlMode) {
            case FIELD_FOLLOWING:
                setControlMode(ControlMode.EJECT);
                break;
            case CAMERA_FOLLOWING:
                setControlMode(ControlMode.CENTER_FOLLOWING);
                break;
            case EJECT:
                setControlMode(ControlMode.CENTER_FOLLOWING);
                break;
            case CENTER_FOLLOWING:
                setControlMode(ControlMode.ABSOLUTE_FOLLOWING);
                break;
            case ABSOLUTE_FOLLOWING:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
            case POSITION:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
            case MANUAL:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
        }
    }

    /** offsets */
    private int fieldFollowingOffset() {
        return -convertTurretDegreesToTicks( // this is currently negated because motor is running counterclockwise
            robotState.fieldToVehicle.getRotation().getDegrees()
        );
    }

    private int cameraFollowingOffset() {
        var delta = -camera.getDeltaX();
        return ((int) (delta * kDeltaXScalar));
    }

    private int targetFollowingOffset() {
        double opposite = target.getY() - robotState.getFieldToTurretPos().getY();
        double adjacent = target.getX() - robotState.getFieldToTurretPos().getX();
        double turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    private int estimatedTargetFollowingOffset() {
        double opposite =
            target.getY() - robotState.getEstimatedFieldToTurretPos().getY();
        double adjacent =
            target.getX() - robotState.getEstimatedFieldToTurretPos().getX();
        double turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    /** actions for modes */
    private void trackGyro() {
        int fieldTickOffset = fieldFollowingOffset();
        int adj = (desiredPos + fieldTickOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void trackTarget() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = targetFollowingOffset();

        int adj = (desiredPos + fieldTickOffset + targetOffset + visionCorroboration);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
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

    private void trackAbsolute() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = estimatedTargetFollowingOffset();

        int adj = (desiredPos + fieldTickOffset + targetOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    private void eject() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = estimatedTargetFollowingOffset();
        int throwOffset = convertTurretDegreesToTicks(30);

        int adj = (desiredPos + fieldTickOffset + targetOffset + throwOffset);
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

            turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, rawPos);
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

    /** config and misc */
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

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Turret/Deadzone", deadzone ? "Deadzone" : "Free");
    }

    /** modes */
    public enum ControlMode {
        CAMERA_FOLLOWING,
        FIELD_FOLLOWING,
        CENTER_FOLLOWING,
        ABSOLUTE_FOLLOWING,
        EJECT,
        POSITION,
        MANUAL,
    }
}
