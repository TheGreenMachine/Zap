package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.auto.modes.AutoModeBase;
import com.team1816.season.controlboard.ControlUtils;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.controlboard.*;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Controller.Factory.class).to(ControlUtils.class);
        requestStaticInjection(TrajectoryAction.class);
        requestStaticInjection(AutoModeBase.class);
        requestStaticInjection(WaitUntilInsideRegion.class);
    }
}
