package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.Superstructure;
import com.team1816.season.subsystems.Shooter;

public class ShootAction implements Action {

    @Inject
    private static Superstructure superstructure;

    @Inject
    private static Shooter shooter;

    private final boolean isShooting;
    private final boolean hoodOut;

    public ShootAction(boolean isShooting, boolean hoodOut) {
        this.isShooting = isShooting;
        this.hoodOut = hoodOut;
    }

    @Override
    public void start() {
        shooter.setHood(hoodOut);
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
