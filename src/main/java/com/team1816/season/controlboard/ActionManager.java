package com.team1816.season.controlboard;

import java.util.Arrays;
import java.util.List;

/*
    Responsible for mapping buttons, triggers, and joystick values represented in each controller
    (GamepadDriveControlBoard and GamepadOperatorController) to specific actions (ie: getShoot, a method in
    GamepadOperatorControlBoard that corresponds to operator controller's right trigger MAPS TO the firing action in
    superstructure)
 */
public class ActionManager {

    private List<ControlUtils.ButtonAction> actions;

    public ActionManager(ControlUtils.ButtonAction... actions) {
        this.actions = Arrays.asList(actions);
        this.update(); //Used to insured actions are in intialized state and doesn't get triggered on enabling
    }

    public void update() {
        actions.forEach(ControlUtils.ButtonAction::update);
    }
}
