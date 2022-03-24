package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.Superstructure;
import com.team1816.season.subsystems.Drive;

public class StopAction implements Action {

    public boolean notRevving;

    @Inject
    private static Superstructure superstructure;

//    @Inject
//    private static Drive.Factory driveFactory;

//    private Drive drive;

    public StopAction(boolean notRevving) {
        this.notRevving = notRevving;
//        drive = driveFactory.getInstance();
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
