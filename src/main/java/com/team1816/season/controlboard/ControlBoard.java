package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.controlboard.*;
import com.team1816.season.Constants;
import com.team1816.season.auto.AutoModeManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard implements IControlBoard {

    private static final ControlBoardBrige controlBoardBridge = ControlBoardBrige.getInstance();

    private final Controller driverController;
    private final Controller operatorController;

    private double demoModeMultiplier;
    private SendableChooser<DemoMode> demoModeChooser;

    @Inject
    private ControlBoard(Controller.Factory controller) {
        driverController = controller.getControllerInstance(Constants.kDriveGamepadPort);
        operatorController =
            controller.getControllerInstance(Constants.kOperatorGamepadPort);

        // only for demo mode functionality
        if(controlBoardBridge.isDemoMode()){
            demoModeChooser = new SendableChooser<>();
            SmartDashboard.putData("Demo mode", demoModeChooser);
            for (DemoMode demoMode : DemoMode.values()) {
                demoModeChooser.addOption(demoMode.name(), demoMode);
            }
            demoModeChooser.setDefaultOption(
                DemoMode.PLAIN.name(),
                DemoMode.PLAIN
            );
            demoModeMultiplier = 0.5;
        }
    }

    @Override
    public boolean getAsBool(String getName) {
        return getBooleanFromControllerYaml(getName);
    }

    @Override
    public double getAsDouble(String getName) {
        if(controlBoardBridge.isDemoMode()){
            return getDoubleFromControllerYaml(getName);
        } else {
            return getDoubleFromControllerYaml(getName) * demoModeMultiplier;
        }
    }

    public double getDoubleFromControllerYaml(String name) {
        return getDoubleFromControllerYaml(name, 0);
    }

    public boolean getBooleanFromControllerYaml(String name) {
        return getBooleanFromControllerYaml(name, false);
    }

    public double getDoubleFromControllerYaml(String name, double defaultVal) {
        if (controlBoardBridge.driverMapContainsKey(name)) {
            if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
                return driverController.getJoystick(
                    controlBoardBridge.getDriverAxisMap().get(name)
                );
            }
            if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                        controlBoardBridge.getDriverButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        } else if (controlBoardBridge.operatorMapContainsKey(name)) {
            if (controlBoardBridge.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getJoystick(
                    controlBoardBridge.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardBridge.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                        controlBoardBridge.getOperatorButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        }

        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        if (controlBoardBridge.driverMapContainsKey(name)) {
            if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
                return driverController.getTrigger(
                    controlBoardBridge.getDriverAxisMap().get(name)
                );
            }
            if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                    controlBoardBridge.getDriverButtonMap().get(name)
                );
            }
        } else if (controlBoardBridge.operatorMapContainsKey(name)) {
            if (controlBoardBridge.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getTrigger(
                    controlBoardBridge.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardBridge.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                    controlBoardBridge.getOperatorButtonMap().get(name)
                );
            }
        }

        return defaultVal;
    }

    private enum DemoMode {
        PLAIN,
        SPORT,
        PLAD
    }
}
