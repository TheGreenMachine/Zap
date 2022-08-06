package com.team1816.season.events;

import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.events.PubSubRunnable;

public class ControlboardEventCollection {

    public static class ToggleCollectorEvent extends PubSubConsumer<Boolean> {}

}
