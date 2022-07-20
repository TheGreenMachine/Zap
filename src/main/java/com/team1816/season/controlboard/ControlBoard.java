package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.controlboard.*;
import com.team1816.season.Constants;

public class ControlBoard implements IControlBoard {

    private static final ControlBoardFactory controlBoardFactory = ControlBoardFactory.getInstance();

    private final Controller driverController;
    private final Controller operatorController;

    @Inject
    private ControlBoard(Controller.Factory controller) {
        driverController = controller.getControllerInstance(Constants.kDriveGamepadPort);
        operatorController =
            controller.getControllerInstance(Constants.kOperatorGamepadPort);
    }

    @Override
    public boolean getAsBool(String getName) {
        return getBooleanFromControllerYaml(getName);
    }

    @Override
    public double getAsDouble(String getName) {
        return getDoubleFromControllerYaml(getName);
    }

    public double getDoubleFromControllerYaml(String name) {
        return getDoubleFromControllerYaml(name, 0);
    }

    public boolean getBooleanFromControllerYaml(String name) {
        return getBooleanFromControllerYaml(name, false);
    }

    public double getDoubleFromControllerYaml(String name, double defaultVal) {
        if (controlBoardFactory.driverMapContainsKey(name)) {
            if (controlBoardFactory.getDriverAxisMap().containsKey(name)) {
                return driverController.getJoystick(
                    controlBoardFactory.getDriverAxisMap().get(name)
                );
            }
            if (controlBoardFactory.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                        controlBoardFactory.getDriverButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        } else if (controlBoardFactory.operatorMapContainsKey(name)) {
            if (controlBoardFactory.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getJoystick(
                    controlBoardFactory.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardFactory.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                        controlBoardFactory.getOperatorButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        }

        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        if (controlBoardFactory.driverMapContainsKey(name)) {
            if (controlBoardFactory.getDriverAxisMap().containsKey(name)) {
                return driverController.getTrigger(
                    controlBoardFactory.getDriverAxisMap().get(name)
                );
            }
            if (controlBoardFactory.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                    controlBoardFactory.getDriverButtonMap().get(name)
                );
            }
        } else if (controlBoardFactory.operatorMapContainsKey(name)) {
            if (controlBoardFactory.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getTrigger(
                    controlBoardFactory.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardFactory.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                    controlBoardFactory.getOperatorButtonMap().get(name)
                );
            }
        }

        return defaultVal;
    }
}
