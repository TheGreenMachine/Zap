package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.subsystems.Shooter;

public class RampUpShooterAction implements Action {

    private static Shooter shooter;

    private static Orchestrator orchestrator;

    private final int shooterVel;

    public RampUpShooterAction(int shooterVel) {
        this.shooterVel = shooterVel;
        shooter = Injector.get(Shooter.class);
        orchestrator = Injector.get(Orchestrator.class);
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
