package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.season.Constants;
import com.team1816.season.subsystems.Turret;

public class AutoAimAction implements Action {

    private static Turret turret;

    private Turret.ControlMode prevControlMode;

    private AsyncTimer aimTimer;

    private double duration;

    public AutoAimAction(double duration) {
        turret = Injector.get(Turret.class);
        this.duration = duration;
    }

    @Override
    public void start() {
        if (Constants.kUseVision) {
            prevControlMode = turret.getControlMode();
            aimTimer =
                new AsyncTimer(
                    duration,
                    () -> turret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING),
                    () -> turret.lockTurret()
                );
            aimTimer.update();
            turret.setControlMode(Turret.ControlMode.CENTER_FOLLOWING);
        }
    }

    @Override
    public void update() {
        if (Constants.kUseVision) {
            aimTimer.update();
        }
    }

    @Override
    public boolean isFinished() {
        if (Constants.kUseVision) {
            return aimTimer.isCompleted();
        } else {
            return true;
        }
    }

    @Override
    public void done() {
        if (Constants.kUseVision) {
            turret.lockTurret();
        }
    }
}
