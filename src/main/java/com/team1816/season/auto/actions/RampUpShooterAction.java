package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.Superstructure;
import com.team1816.season.subsystems.Shooter;

public class RampUpShooterAction implements Action {

    @Inject
    private static Shooter shooter;

    @Inject
    private static Superstructure superstructure;

    private final int shooterVel;

    public RampUpShooterAction(int shooterVel) {
        this.shooterVel = shooterVel;
    }

    @Override
    public void start() {
        superstructure.setRevving(shooterVel > 0, shooterVel);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return shooter.isVelocityNearTarget();
    }

    @Override
    public void done() {}
}
