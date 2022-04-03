package com.team1816.season.auto.actions;

import com.google.inject.Inject;
import com.team1816.lib.auto.actions.Action;
import com.team1816.season.subsystems.Turret;

public class TurretAction implements Action {

    @Inject
    private static Turret turret;

    private double turretAngle;

    public TurretAction(double turretAngle) {
        this.turretAngle = turretAngle;
    }

    public TurretAction(Turret.ControlMode mode) {
        turret.setControlMode(mode);
    }

    @Override
    public void start() {
        turret.setTurretAngle(turretAngle);
        System.out.println("setting turret angle to " + turretAngle);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Math.abs(turret.getPositionError()) < turret.ALLOWABLE_ERROR_TICKS && turret.getControlMode() == Turret.ControlMode.POSITION;
    }

    @Override
    public void done() {}
}
