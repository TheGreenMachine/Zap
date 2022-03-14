package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.Superstructure;

public class StopAction implements Action {

    public boolean notRevving;

    @Inject
    private static Superstructure superstructure;

    public StopAction(boolean notRevving) {
        this.notRevving = notRevving;
    }

    @Override
    public void start() {
        superstructure.setStopped(notRevving);
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
