package com.team1816.season.events;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.events.EventAggregator;
import com.team1816.lib.events.PubSubRunnable;
import com.team1816.lib.subsystems.Drive;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;

/**
 * List of all used events
 */

@Singleton
public class EventRegister {
    private EventAggregator eventManager;

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
        eventManager.GetEvent(EventRegister.IncrementBucketEvent.class).Subscribe(
            () -> {
                distanceManager.incrementBucket(-100);
            }
        );

    }

    public static class IncrementBucketEvent extends PubSubRunnable {}

}
