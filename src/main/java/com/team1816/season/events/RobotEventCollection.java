package com.team1816.season.events;

import com.google.inject.Singleton;
import com.team1816.lib.events.EventBase;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.events.PubSubRunnable;

import java.util.HashMap;

@Singleton
public class RobotEventCollection {
    public static HashMap<String, EventBase> map = new HashMap<>();

    public RobotEventCollection () {
        for(Class c: ControlBoardEventCollection.class.getClasses()) {
            map.put(c.getName(), EventBase.class.cast(c));
        }
    }

    public static class ToggleCollectorEvent extends PubSubConsumer<Boolean> {}

    public static class ToggleCollectorReverseEvent extends PubSubConsumer<Boolean> {}

    public static class UnlockClimberEvent extends PubSubRunnable {}

    public static class ToggleManualShootEvent extends PubSubRunnable {}

    public static class ZeroPoseEvent extends PubSubRunnable {}

    public static class BrakeModeEvent extends PubSubConsumer<Boolean> {}

    public static class SlowModeEvent extends PubSubConsumer<Boolean> {}

    public static class IncrementBucketEvent extends PubSubRunnable {}

    public static class DecrementBucketEvent extends PubSubRunnable {}

    public static class AutoAimEvent extends PubSubConsumer<Boolean> {}

    public static class ToggleCameraEvent extends PubSubRunnable {}

    public static class YeetShotEvent extends PubSubConsumer<Boolean> {}

    public static class ShootEvent extends PubSubConsumer<Boolean> {}

    public static class TurretJogLeftEvent extends PubSubConsumer<Boolean> {}

    public static class TurretJogRightEvent extends PubSubConsumer<Boolean> {}

    public static class ManualClimberArmUpEvent extends PubSubConsumer<Boolean> {}

    public static class ManualClimberArmDownEvent extends PubSubConsumer<Boolean> {}

    public static class ToggleTopClampEvent extends PubSubRunnable {}

    public static class ToggleBottomClampEvent extends PubSubRunnable {}

    public static class AutoClimbEvent extends PubSubRunnable {}
}
