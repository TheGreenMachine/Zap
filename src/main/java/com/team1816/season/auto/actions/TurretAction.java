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

    @Override
    public void start() {
        turret.setTurretAngle(turretAngle);
        // turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return turret.getPositionError() < 10;
    }

    @Override
    public void done() {}
}
