package com.team1816.lib.controlboard;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;

// This is a bridging class that allows for yaml functionality and it's sole purpose is to map controls to their respective methods
public class ControlBoardBridge {

    private ControlBoardConfig config;

    private HashMap<String, Controller.Button> driverButtonMap = new HashMap<>();
    private HashMap<String, Controller.Axis> driverAxisMap = new HashMap<>();
    private HashMap<String, Integer> driverDpadMap = new HashMap<>();
    private boolean driverRumble = false;

    private HashMap<String, Controller.Button> operatorButtonMap = new HashMap<>();
    private HashMap<String, Controller.Axis> operatorAxisMap = new HashMap<>();
    private HashMap<String, Integer> operatorDpadMap = new HashMap<>();
    private boolean operatorRumble = false;

    public ControlBoardBridge() {
        var controlBoardConfigName = "example"; //TODO read this from somewhere probably going to be robotConfig yaml
        try {
            //            File file = new File(
            //                "src/main/resources/" +
            //                controlBoardConfigName +
            //                ".controlboard.config.yml"
            //            );
            //            config = ControlBoardYamlConfig.loadFrom(new FileInputStream(file));
            config =
                ControlBoardYamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(controlBoardConfigName + ".config.yml")
                );
            System.out.println(
                "Loading " + controlBoardConfigName + " control board config"
            );
        } catch (Exception e) {
            System.out.println(e);
            DriverStation.reportError(
                "Control Board Yaml Config error!",
                e.getStackTrace()
            );
        }

        //TODO: make sure that config is not null and actually does read from yaml
        if (config != null) {
            if (config.driver != null) {
                if (config.driver.rumble != null) {
                    driverRumble = config.driver.rumble;
                }
                if (config.driver.joysticks != null) {
                    if (config.driver.joysticks.get("left") != null) {
                        if (config.driver.joysticks.get("left").horizontal != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("left").horizontal,
                                Controller.Axis.LEFT_X
                            );
                        }
                        if (config.driver.joysticks.get("left").vertical != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("left").vertical,
                                Controller.Axis.LEFT_Y
                            );
                        }
                    }
                    if (config.driver.joysticks.get("right") != null) {
                        if (config.driver.joysticks.get("right").horizontal != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("right").horizontal,
                                Controller.Axis.RIGHT_X
                            );
                        }
                        if (config.driver.joysticks.get("right").vertical != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("right").vertical,
                                Controller.Axis.RIGHT_Y
                            );
                        }
                    }
                }
                if (config.driver.axes != null) {
                    driverAxisMap.put(
                        config.driver.axes.getOrDefault("leftTrigger", "empty"),
                        Controller.Axis.LEFT_TRIGGER
                    );
                    driverAxisMap.put(
                        config.driver.axes.getOrDefault("rightTrigger", "empty"),
                        Controller.Axis.RIGHT_TRIGGER
                    );
                }
                if (config.driver.buttonpad != null) {
                    if (config.driver.buttonpad.x != null) driverButtonMap.put(
                        config.driver.buttonpad.x,
                        Controller.Button.X
                    );
                    if (config.driver.buttonpad.y != null) driverButtonMap.put(
                        config.driver.buttonpad.y,
                        Controller.Button.Y
                    );
                    if (config.driver.buttonpad.a != null) driverButtonMap.put(
                        config.driver.buttonpad.a,
                        Controller.Button.A
                    );
                    if (config.driver.buttonpad.b != null) driverButtonMap.put(
                        config.driver.buttonpad.b,
                        Controller.Button.B
                    );
                }
                if (config.driver.buttons != null) {
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("leftBumper", "empty"),
                        Controller.Button.LEFT_BUMPER
                    );
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("rightBumper", "empty"),
                        Controller.Button.RIGHT_BUMPER
                    );

                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("start", "empty"),
                        Controller.Button.START
                    );
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("back", "empty"),
                        Controller.Button.BACK
                    );
                }
                if (config.driver.dpad != null) {
                    if (config.driver.dpad.up != null) driverDpadMap.put(
                        config.driver.dpad.up,
                        0
                    );
                    if (config.driver.dpad.right != null) driverDpadMap.put(
                        config.driver.dpad.right,
                        90
                    );
                    if (config.driver.dpad.down != null) driverDpadMap.put(
                        config.driver.dpad.down,
                        180
                    );
                    if (config.driver.dpad.left != null) driverDpadMap.put(
                        config.driver.dpad.up,
                        270
                    );
                }
            }
            if (config.operator != null) {
                if (config.operator.rumble != null) {
                    operatorRumble = config.operator.rumble;
                }
                if (config.operator.joysticks != null) {
                    if (config.operator.joysticks.get("left") != null) {
                        if (config.operator.joysticks.get("left").horizontal != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("left").horizontal,
                                Controller.Axis.LEFT_X
                            );
                        }
                        if (config.operator.joysticks.get("left").vertical != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("left").vertical,
                                Controller.Axis.LEFT_Y
                            );
                        }
                    }
                    if (config.operator.joysticks.get("right") != null) {
                        if (config.operator.joysticks.get("right").horizontal != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("right").horizontal,
                                Controller.Axis.RIGHT_X
                            );
                        }
                        if (config.operator.joysticks.get("right").vertical != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("right").vertical,
                                Controller.Axis.RIGHT_Y
                            );
                        }
                    }
                }
                if (config.operator.axes != null) {
                    operatorAxisMap.put(
                        config.operator.axes.getOrDefault("leftTrigger", "empty"),
                        Controller.Axis.LEFT_TRIGGER
                    );
                    operatorAxisMap.put(
                        config.operator.axes.getOrDefault("rightTrigger", "empty"),
                        Controller.Axis.RIGHT_TRIGGER
                    );
                }
                if (config.operator.buttonpad != null) {
                    if (config.operator.buttonpad.x != null) operatorButtonMap.put(
                        config.operator.buttonpad.x,
                        Controller.Button.X
                    );
                    if (config.operator.buttonpad.y != null) operatorButtonMap.put(
                        config.operator.buttonpad.y,
                        Controller.Button.Y
                    );
                    if (config.operator.buttonpad.a != null) operatorButtonMap.put(
                        config.operator.buttonpad.a,
                        Controller.Button.A
                    );
                    if (config.operator.buttonpad.b != null) operatorButtonMap.put(
                        config.operator.buttonpad.b,
                        Controller.Button.B
                    );
                }
                if (config.operator.buttons != null) {
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("leftBumper", "empty"),
                        Controller.Button.LEFT_BUMPER
                    );
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("rightBumper", "empty"),
                        Controller.Button.RIGHT_BUMPER
                    );

                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("start", "empty"),
                        Controller.Button.START
                    );
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("back", "empty"),
                        Controller.Button.BACK
                    );
                }
                if (config.operator.dpad != null) {
                    if (config.operator.dpad.up != null) operatorDpadMap.put(
                        config.operator.dpad.up,
                        0
                    );
                    if (config.operator.dpad.right != null) operatorDpadMap.put(
                        config.operator.dpad.right,
                        90
                    );
                    if (config.operator.dpad.down != null) operatorDpadMap.put(
                        config.operator.dpad.down,
                        180
                    );
                    if (config.driver.dpad.left != null) operatorDpadMap.put(
                        config.operator.dpad.up,
                        270
                    );
                }
            }
        }
    }

    public HashMap<String, Controller.Button> getDriverButtonMap() {
        return driverButtonMap;
    }

    public HashMap<String, Controller.Axis> getDriverAxisMap() {
        return driverAxisMap;
    }

    public HashMap<String, Integer> getDriverDpadMap() {
        return driverDpadMap;
    }

    public HashMap<String, Controller.Button> getOperatorButtonMap() {
        return operatorButtonMap;
    }

    public HashMap<String, Controller.Axis> getOperatorAxisMap() {
        return operatorAxisMap;
    }

    public HashMap<String, Integer> getOperatorDpadMap() {
        return operatorDpadMap;
    }
}
