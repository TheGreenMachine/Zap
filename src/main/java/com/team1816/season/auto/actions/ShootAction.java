package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Superstructure;

public class ShootAction implements Action {

    @Inject
    private static Superstructure superstructure;

    private final boolean isShooting;

    public ShootAction(boolean isShooting, boolean hoodOut) {
        this.isShooting = isShooting;
    }

    @Override
    public void start() {
        superstructure.setFiring(isShooting); // if using camera, setHood is overridden
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
