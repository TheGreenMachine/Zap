package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Superstructure;

public class StopAction implements Action {

    public boolean notRevving;

    private static Superstructure superstructure;

    public StopAction(boolean notRevving) {
        this.notRevving = notRevving;
        superstructure = Injector.get(Superstructure.class);
    }

    @Override
    public void start() {
        System.out.println("stopping action : - - - -");
        superstructure.setStopped(notRevving);
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
