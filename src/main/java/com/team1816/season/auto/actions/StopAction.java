package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Orchestrator;

public class StopAction implements Action {

    public boolean notRevving;

    private static Orchestrator orchestrator;

    public StopAction(boolean notRevving) {
        this.notRevving = notRevving;
        orchestrator = Injector.get(Orchestrator.class);
    }

    @Override
    public void start() {
        System.out.println("stopping action : - - - -");
        orchestrator.setStopped(notRevving);
        //        drive.stop();
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
