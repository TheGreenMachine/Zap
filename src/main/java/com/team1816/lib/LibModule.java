package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.frcSeason.controlboard.ControlUtils;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.controlboard.*;
import com.team1816.lib.subsystems.RobotStateEstimator;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Controller.Factory.class).to(ControlUtils.class);
        requestStaticInjection(DriveTrajectory.class);
        requestStaticInjection(RobotStateEstimator.class);
        requestStaticInjection(WaitUntilInsideRegion.class);
    }
}
