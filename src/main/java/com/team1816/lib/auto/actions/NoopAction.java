package com.team1816.lib.auto.actions;

/**
 * The NoopAction class is an instance of the Action interface and is empty. Performs nothing, old class carried over.
 */
public class NoopAction implements Action {

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
