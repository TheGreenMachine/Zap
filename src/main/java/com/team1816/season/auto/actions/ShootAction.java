package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.subsystems.Orchestrator;
import com.team1816.season.subsystems.Shooter;

public class ShootAction implements Action {

    @Inject
    private static Orchestrator orchestrator;

    @Inject
    private static Shooter shooter;

    private boolean isShooting;
    private boolean hoodOut;

    public ShootAction(boolean isShooting, boolean hoodOut) {
        this.isShooting = isShooting;
        this.hoodOut = hoodOut;
    }

    @Override
    public void start() {
        orchestrator.setFiring(isShooting);
        shooter.setHood(hoodOut);
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
