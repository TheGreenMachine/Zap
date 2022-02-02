package com.team1816.season.controlboard;

import java.util.Arrays;
import java.util.List;

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
