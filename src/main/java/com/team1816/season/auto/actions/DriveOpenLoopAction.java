package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Drive;

public class DriveOpenLoopAction implements Action {

    @Inject
    private static Drive.Factory driveFactory;

    private static Drive drive;

    private final AsyncTimer driveTimer;

    public DriveOpenLoopAction(double driveTime, double percentOutput) {
        drive = driveFactory.getInstance();
        this.driveTimer =
            new AsyncTimer(
                driveTime,
                () -> drive.setTeleopInputs(percentOutput, 0, 0),
                () -> drive.setTeleopInputs(0, 0, 0)
            );
    }

    @Override
    public void start() {
        driveTimer.update();
    }

    @Override
    public void update() {
        driveTimer.update();
    }

    @Override
    public boolean isFinished() {
        return driveTimer.isCompleted();
    }

    @Override
    public void done() {
        drive.setTeleopInputs(0, 0, 0);
    }
}
