package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.Superstructure;

public class CollectAction implements Action {

    private final boolean isCollecting;

    @Inject
    private static Superstructure superstructure;

    public CollectAction(boolean isCollecting) {
        this.isCollecting = isCollecting;
    }

    @Override
    public void start() {
        System.out.println("Modifying collector!");
        superstructure.setCollecting(isCollecting, false); // hard coded false because we shouldn't ever backSpin
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
