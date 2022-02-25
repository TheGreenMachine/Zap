package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.subsystems.Orchestrator;
import com.team1816.season.subsystems.Shooter;

public class RampUpShooterAction implements Action {

    @Inject
    private static Shooter shooter;

    @Inject
    private static Orchestrator orchestrator;

    private final int shooterVel;

    public RampUpShooterAction(int shooterVel) {
        this.shooterVel = shooterVel;
    }

    @Override
    public void start() {
        orchestrator.setRevving(shooterVel > 0, shooterVel);
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
