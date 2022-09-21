package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.controlboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class ControlBoard implements IControlBoard {

    // Control Board
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorGamepadPort = 1;

    private static final ControlBoardBrige controlBoardBridge = ControlBoardBrige.getInstance();

    private final Controller driverController;
    private final Controller operatorController;

    // For demo only
    private double demoModeMultiplier;
    private SendableChooser<DemoMode> demoModeChooser;
    private DemoMode desiredMode;

    @Inject
    private ControlBoard(Controller.Factory controller) {
        driverController = controller.getControllerInstance(kDriveGamepadPort);
        operatorController = controller.getControllerInstance(kOperatorGamepadPort);

        // demo mode functionality configuration
        if (controlBoardBridge.isDemoMode()) {
            System.out.println("Using Demo Control Board");

            demoModeChooser = new SendableChooser<>();
            SmartDashboard.putData("Demo Mode", demoModeChooser);
            for (DemoMode demoMode : DemoMode.values()) {
                demoModeChooser.addOption(demoMode.name(), demoMode);
            }
            demoModeChooser.setDefaultOption(DemoMode.SLOW.name(), DemoMode.SLOW);
            demoModeMultiplier = 0.25;
        }
    }

    @Override
    public boolean getAsBool(String getName) {
        return getBooleanFromControllerYaml(getName);
    }

    @Override
    public double getAsDouble(String getName) {
        if (controlBoardBridge.isDemoMode()) {
            return getDoubleFromControllerYaml(getName) * demoModeMultiplier;
        } else {
            return getDoubleFromControllerYaml(getName);
        }
    }

    @Override
    public boolean update() {
        DemoMode selectedMode = demoModeChooser.getSelected();
        boolean modeChanged = desiredMode != selectedMode;

        // if auto has been changed, update selected auto mode + thread
        if (modeChanged) {
            System.out.println(
                "Demo mode changed from: " + desiredMode + ", to: " + selectedMode.name()
            );

            switch (selectedMode) {
                case SLOW:
                    demoModeMultiplier = 0.25;
                    break;
                case COMFORT:
                    demoModeMultiplier = 0.5;
                    break;
                case SPORT:
                    demoModeMultiplier = 0.75;
                    break;
                case PLAID:
                    demoModeMultiplier = 1;
                    break;
            }
        }
        desiredMode = selectedMode;

        return modeChanged;
    }

    public void outputToSmartDashboard() {
        if (desiredMode != null) {
            SmartDashboard.putString("Selected Demo Mode", desiredMode.name());
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
            } else if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                    controlBoardBridge.getDriverButtonMap().get(name)
                );
            } else if (controlBoardBridge.getDriverDpadMap().containsKey(name)) {
                return (
                    driverController.getDPad() ==
                    controlBoardBridge.getDriverDpadMap().get(name)
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
            } else if (controlBoardBridge.getOperatorDpadMap().containsKey(name)) {
                return (
                    operatorController.getDPad() ==
                    controlBoardBridge.getOperatorDpadMap().get(name)
                );
            }
        }

        return defaultVal;
    }

    private enum DemoMode {
        SLOW,
        COMFORT,
        SPORT,
        PLAID,
    }
}
