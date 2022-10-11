package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Orchestrator;

public class ShootAction implements Action {

    private static Orchestrator orchestrator;

    private final boolean isShooting;

    public ShootAction(boolean isShooting, boolean hoodOut) {
        this.isShooting = isShooting;
        orchestrator = Injector.get(Orchestrator.class);
    }

    @Override
    public void start() {
        orchestrator.setFiring(isShooting); // if using camera, setHood is overridden
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
