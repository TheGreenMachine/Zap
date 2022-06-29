package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class GhostMotor implements IGreenMotor, IMotorSensor {

    private ControlMode controlMode;
    private final int maxVelTicks100ms;
    private final double[] demand = new double[] { 0, 0 };

    protected double lastSet = Double.NaN;

    protected String name = "";
    private final int absInitOffset;

    public GhostMotor(int maxTickVel, int absInitOffset, String motorName) {
        this.absInitOffset = absInitOffset;
        maxVelTicks100ms = maxTickVel;
        name = motorName;
    }

    @Override
    public void set(ControlMode Mode, double demand) {
        processSet(Mode, demand);
    }

    @Override
    public void set(
        ControlMode Mode,
        double demand0,
        DemandType demand1Type,
        double demand1
    ) {
        processSet(Mode, demand0);
    }

    private void processSet(ControlMode Mode, double demand) {
        if (
            Mode == ControlMode.Position ||
            Mode == ControlMode.Velocity ||
            Mode == ControlMode.PercentOutput
        ) {
            this.demand[0] = demand;
            controlMode = Mode;
        }
    }

    @Override
    public void neutralOutput() {}

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {}

    @Override
    public void setSensorPhase(boolean PhaseSensor) {}

    @Override
    public void setInverted(boolean invert) {}

    @Override
    public void setInverted(InvertType invertType) {}

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedloopRamp(
        double secondsFromNeutralToFull,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(
        int filterWindowSamples,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {}

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double getMotorOutputPercent() {
        return 0;
    }

    @Override
    public double getMotorOutputVoltage() {
        return 0;
    }

    @Override
    @Deprecated
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(
        SensorVelocityMeasPeriod period,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double getTemperature() {
        return 0;
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(
        RemoteFeedbackDevice feedbackDevice,
        int pidIdx,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(
        double coefficient,
        int pidIdx,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        int deviceID,
        RemoteSensorSource remoteSensorSource,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        CANCoder canCoderRef,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        BaseTalon talonRef,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configSensorTerm(
        SensorTerm sensorTerm,
        FeedbackDevice feedbackDevice,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        return demand[pidIdx];
    }

    @Override
    public double getSelectedSensorVelocity(int pidIdx) {
        var output = demand[pidIdx];
        if (controlMode == ControlMode.PercentOutput) {
            if (Math.abs(output) > 1.1) {
                System.out.println(
                    "Motor " +
                    name +
                    "'s % output should be between -1.0 to 1.0 value:" +
                    output
                );
            }
            return output * maxVelTicks100ms;
        }
        return output;
    }

    @Override
    public ErrorCode setSelectedSensorPosition(
        double sensorPos,
        int pidIdx,
        int timeoutMs
    ) {
        demand[pidIdx] = sensorPos;
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrame frame,
        int periodMs,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(
        RemoteLimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int deviceID,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        RemoteLimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int deviceID,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {}

    @Override
    public ErrorCode configForwardSoftLimitThreshold(
        double forwardSensorLimit,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(
        double reverseSensorLimit,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {}

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(
        int slotIdx,
        double allowableCloseLoopError,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(
        int slotIdx,
        double iaccum,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(
        int slotIdx,
        double percentOut,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public double getClosedLoopError(int pidIdx) {
        return 0;
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        return 0;
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        return 0;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {}

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        return 0;
    }

    @Override
    public double getActiveTrajectoryPosition() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        return 0;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(
        double sensorUnitsPer100ms,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionAcceleration(
        double sensorUnitsPer100msPerSec,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(
        int baseTrajDurationMs,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        return ErrorCode.OK;
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        return 0;
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return ErrorCode.OK;
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        return false;
    }

    @Override
    public void processMotionProfileBuffer() {}

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getLastError() {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public int getFirmwareVersion() {
        return 0;
    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(
        ParamEnum param,
        double value,
        int subValue,
        int ordinal,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSetParameter(
        int param,
        double value,
        int subValue,
        int ordinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public int getBaseID() {
        return 0;
    }

    @Override
    public int getDeviceID() {
        return -1;
    }

    @Override
    public ControlMode getControlMode() {
        return null;
    }

    @Override
    public void follow(IMotorController masterToFollow) {}

    @Override
    public void valueUpdated() {}

    @Override
    public ErrorCode configSelectedFeedbackSensor(
        FeedbackDevice feedbackDevice,
        int pidIdx,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(
        VelocityMeasPeriod period,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }

    @Override
    public int getQuadraturePosition() {
        return (int) demand[0];
    }

    @Override
    public int getPulseWidthPosition() {
        return absInitOffset;
    }

    @Override
    public ErrorCode setQuadraturePosition(int newPosition) {
        demand[0] = newPosition;
        return ErrorCode.OK;
    }

    @Override
    public double getLastSet() {
        return lastSet;
    }

    @Override
    public String getName() {
        return name;
    }
}
