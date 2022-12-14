package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Orchestrator;

public class CollectAction implements Action {

    private final boolean isCollecting;

    private static Orchestrator orchestrator;

    public CollectAction(boolean isCollecting) {
        this.isCollecting = isCollecting;
        orchestrator = Injector.get(Orchestrator.class);
    }

    @Override
    public void start() {
        System.out.println("Modifying collector!");
        orchestrator.setCollecting(isCollecting, true); // hard coded false because we shouldn't ever backSpin
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
