package com.team1816.season;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.season.controlboard.ControlBoard;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        // only bind items to season class in this module
        bind(IControlBoard.class).to(ControlBoard.class);
    }
}
