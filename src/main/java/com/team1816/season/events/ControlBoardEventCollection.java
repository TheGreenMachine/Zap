package com.team1816.season.events;

import com.team1816.lib.events.PubSubRunnable;

public class ControlBoardEventCollection {

    public static class ToggleCollectorEvent extends PubSubRunnable {}

    public static class ToggleCollectorReverseEvent extends PubSubRunnable {}

    public static class UnlockClimberEvent extends PubSubRunnable {}

    public static class ToggleManualShootEvent extends PubSubRunnable {}

    public static class ZeroPoseEvent extends PubSubRunnable {}

    public static class BrakeModeEvent extends PubSubRunnable {}

    public static class SlowModeEvent extends PubSubRunnable {}

    public static class IncrementBucketEvent extends PubSubRunnable {}

    public static class DecrementBucketEvent extends PubSubRunnable {}

    public static class AutoAimEvent extends PubSubRunnable {}

    public static class ToggleCameraEvent extends PubSubRunnable {}

    public static class YeetShotEvent extends PubSubRunnable {}

    public static class ShootEvent extends PubSubRunnable {}

    public static class TurretJogLeftEvent extends PubSubRunnable {}

    public static class TurretJogRightEvent extends PubSubRunnable {}

    public static class ManualClimberArmUpEvent extends PubSubRunnable {}

    public static class ManualClimberArmDownEvent extends PubSubRunnable {}

    public static class ToggleTopClampEvent extends PubSubRunnable {}

    public static class ToggleBottomClampEvent extends PubSubRunnable {}

    public static class AutoClimbEvent extends PubSubRunnable {}
}
