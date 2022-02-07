package com.team1816.season.auto.modes.modes2020;

import com.team1816.season.auto.actions.actions2020.TurretAction;
import com.team1816.season.subsystems.Turret;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoModeBase;

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
