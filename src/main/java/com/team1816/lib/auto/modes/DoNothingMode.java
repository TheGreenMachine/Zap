package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;

public class DoNothingMode extends AutoMode {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("doing nothing");
    }
}
