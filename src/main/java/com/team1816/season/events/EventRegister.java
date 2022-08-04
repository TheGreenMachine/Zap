package com.team1816.season.events;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.events.EventAggregator;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.events.PubSubRunnable;
import com.team1816.lib.subsystems.Drive;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;
import java.util.function.Consumer;

/**
 * List of all used events
 */

@Singleton
public class EventRegister {

    public static EventAggregator eventManager;

    @Inject
    private static Superstructure superstructure;

    @Inject
    private static Drive.Factory driveFactory;

    private static Drive drive;

    @Inject
    private static Turret turret;

    @Inject
    private static Collector collector;

    @Inject
    private static Spindexer spindexer;

    @Inject
    private static Elevator elevator;

    @Inject
    private static Shooter shooter;

    @Inject
    private static LedManager ledManager;

    @Inject
    private static DistanceManager distanceManager;

    @Inject
    private static Camera camera;

    public EventRegister() {
        eventManager = new EventAggregator();
        drive = driveFactory.getInstance();
        eventManager
            .GetEvent(ToggleCollectorEvent.class)
            .Subscribe(
                (Consumer<Boolean>) pressed -> {
                    superstructure.setCollecting(pressed, true);
                }
            ); // this needs to accept boolean consumers
        eventManager
            .GetEvent(IncrementBucketEvent.class)
            .Subscribe(
                () -> {
                    distanceManager.incrementBucket(100);
                }
            );
    }

    public static class ToggleCollectorEvent extends PubSubConsumer {}

    public static class IncrementBucketEvent extends PubSubRunnable {}
}
