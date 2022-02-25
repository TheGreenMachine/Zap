package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.auto.actions.TurretAction;
import com.team1816.season.subsystems.Turret;

public class TurretTestMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new TurretAction(Turret.CARDINAL_SOUTH),
                new WaitAction(3),
                new TurretAction(Turret.CARDINAL_WEST),
                new WaitAction(3),
                new TurretAction(Turret.CARDINAL_NORTH)
            )
        );
    }
}
