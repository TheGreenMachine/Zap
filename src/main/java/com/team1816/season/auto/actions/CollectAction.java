package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Orchestrator;
import com.team1816.season.subsystems.Spindexer;

public class CollectAction implements Action {

    private boolean isCollecting;

    @Inject
    private static Orchestrator orchestrator;

    public CollectAction(boolean isCollecting) {
        this.isCollecting = isCollecting;
    }

    @Override
    public void start() {
        System.out.println("Modifying collector!");
        orchestrator.setCollecting(isCollecting);
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
