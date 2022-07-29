package com.team1816.season.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.controlboard.*;
import com.team1816.season.Constants;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
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

        // demo mode functionality configuration
        if (controlBoardBridge.isDemoMode()) {
            System.out.println("Using Demo Control Board");
            demoModeChooser = new SendableChooser<>();
            SmartDashboard.putData("Demo mode", demoModeChooser);
            for (DemoMode demoMode : DemoMode.values()) {
                demoModeChooser.addOption(demoMode.name(), demoMode);
            }
            demoModeChooser.setDefaultOption(DemoMode.SLOW.name(), DemoMode.SLOW);
            NetworkTableInstance
                .getDefault()
                .getTable("SmartDashboard")
                .getSubTable("DemoMode")
                .addEntryListener(
                    "selected",
                    (table, key, entry, value, flags) -> {
                        switch ((DemoMode) value.getValue()) {
                            case SLOW:
                                demoModeMultiplier = 0.25;
                            case COMFORT:
                                demoModeMultiplier = 0.5;
                                break;
                            case SPORT:
                                demoModeMultiplier = 0.75;
                                break;
                            case PLAID:
                                demoModeMultiplier = 1.0;
                                break;
                            default:
                                demoModeMultiplier = 0.5;
                        }
                    },
                    EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
                );
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
        SLOW,
        COMFORT,
        SPORT,
        PLAID,
    }
}
