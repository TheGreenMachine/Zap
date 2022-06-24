package com.team1816.lib.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import edu.wpi.first.wpilibj.DriverStation;

public class MotorUtil {

    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }

    public static double getSupplyCurrent(IGreenMotor motor) {
        // If only CTRE had these methods in the interface...
        if (motor instanceof TalonFX) {
            return ((TalonFX) motor).getSupplyCurrent();
        } else if (motor instanceof TalonSRX) {
            return ((TalonSRX) motor).getSupplyCurrent();
        }
        return 0;
    }
}
