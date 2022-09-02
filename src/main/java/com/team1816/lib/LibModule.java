package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.*;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.season.controlboard.ControlUtils;

public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(DriveFactory.class);
        bind(Controller.Factory.class).to(ControlUtils.class);
    }
}
