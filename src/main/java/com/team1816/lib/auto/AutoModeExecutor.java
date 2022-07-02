package com.team1816.lib.auto;

import com.team1816.lib.auto.modes.AutoMode;
import javax.inject.Singleton;

/**
 * This class selects, runs, and (if necessary) stops a specified autonomous mode.
 */
@Singleton
public class AutoModeExecutor {

    private AutoMode autoMode = null;
    private Thread thread = null;

    public AutoModeExecutor() {}

    public void setAutoMode(AutoMode new_auto_mode) {
        autoMode = new_auto_mode;
        thread = new Thread(() -> autoMode.run());
    }

    public void start() {
        if (thread != null) {
            thread.start();
        }
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        autoMode = null;
    }

    public void stop() {
        if (autoMode != null) {
            autoMode.stop();
        }

        thread = null;
    }

    public AutoMode getAutoMode() {
        return autoMode;
    }
}
