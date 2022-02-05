package com.team1816.lib.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.team1816.season.Robot;
import com.team1816.lib.subsystems.Subsystem;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class EnhancedMotorChecker {

    public static class CheckerConfig {

        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.2;

        public static CheckerConfig getForSubsystemMotor(
            Subsystem subsystem,
            IMotorControllerEnhanced motor
        ) {
            var name = subsystem.getSubsystemName();
            var factory = Robot.getFactory();
            return new EnhancedMotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = factory.getConstant(name, "currentFloorCheck");
                    mRPMFloor = factory.getConstant(name, "rpmFloorCheck");
                    mCurrentEpsilon = factory.getConstant(name, "currentEpsilonCheck");
                    mRPMEpsilon = factory.getConstant(name, "rpmEpsilonCheck");
                    mRPMSupplier = () -> motor.getSelectedSensorVelocity(0);
                }
            };
        }
    }

    public static class NamedMotor {

        public String name;
        public IMotorControllerEnhanced motor;

        public NamedMotor(String name, IMotorControllerEnhanced motor) {
            this.name = name;
            this.motor = motor;
        }
    }

    public static boolean checkMotors(
        Subsystem subsystem,
        CheckerConfig checkerConfig,
        NamedMotor... motorsToCheck
    ) {
        boolean failure = false;
        System.out.println("////////////////////////////////////////////////");
        System.out.println(
            "Checking subsystem " +
            subsystem.getClass() +
            " for " +
            motorsToCheck.length +
            " motors."
        );

        List<Double> currents = new ArrayList<>();
        List<Double> rpms = new ArrayList<>();
        List<ControlMode> storedControlModes = new ArrayList<>();

        // Record previous configuration for all motors.
        for (NamedMotor config : motorsToCheck) {
            if (config.motor.getDeviceID() < 0) continue;
            IMotorControllerEnhanced motor = config.motor;

            storedControlModes.add(motor.getControlMode());

            // Now set to disabled.
            motor.set(ControlMode.PercentOutput, 0.0);
        }

        for (NamedMotor config : motorsToCheck) {
            System.out.println("Checking: " + config.name);

            if (config.motor.getDeviceID() < 0) {
                System.out.println("Motor Disabled, Checks Skipped!!");
                continue;
            }

            config.motor.set(
                ControlMode.PercentOutput,
                checkerConfig.mRunOutputPercentage
            );
            Timer.delay(checkerConfig.mRunTimeSec);

            // Now poll the interesting information.
            double current = MotorUtil.getSupplyCurrent(config.motor);
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.print('\n');

            config.motor.set(ControlMode.PercentOutput, 0.0);

            // And perform checks.
            if (current < checkerConfig.mCurrentFloor) {
                DriverStation.reportError(
                    config.name +
                    " has failed current floor check vs " +
                    checkerConfig.mCurrentFloor +
                    "!!!!!!!!!!!!",
                    false
                );
                failure = true;
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    DriverStation.reportError(
                        config.name +
                        " has failed rpm floor check vs " +
                        checkerConfig.mRPMFloor +
                        "!!!!!!!!!!!!!",
                        false
                    );
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // Now run aggregate checks.

        if (currents.size() > 0) {
            Double average = currents
                .stream()
                .mapToDouble(val -> val)
                .average()
                .getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                DriverStation.reportError("Currents varied!!!!!!!!!!!", false);
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            Double average = rpms
                .stream()
                .mapToDouble(val -> val)
                .average()
                .getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                DriverStation.reportError("RPMs varied!!!!!!!!", false);
                failure = true;
            }
        }

        // Restore Talon configurations
        for (int i = 0; i < motorsToCheck.length; ++i) {
            IMotorControllerEnhanced motor = motorsToCheck[i].motor;
            if (motor.getDeviceID() >= 0) {
                motor.set(storedControlModes.get(i), 0);
            }
        }

        return !failure;
    }
}
