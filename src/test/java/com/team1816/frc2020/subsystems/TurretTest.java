package com.team1816.frc2020.subsystems;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.season.RobotState;
import com.team1816.season.SeasonModule;
import com.team1816.season.subsystems.Turret;
import com.team1816.lib.LibModule;
import com.team254.lib.geometry.Rotation2d;
import junit.framework.TestCase;

public class TurretTest extends TestCase {

    private Turret target;
    private Injector injector;

    @Override
    public void setUp() throws Exception {
        target = injector.getInstance(Turret.class);
    }

    public TurretTest() {
        injector = Guice.createInjector(new LibModule(), new SeasonModule());
    }

    public void testSetTurretFieldFollowing() {
        target.setTurretAngle(Turret.CARDINAL_SOUTH);
        target.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        target.writePeriodicOutputs();
        assertEquals(
            (double) Turret.ABS_TICKS_SOUTH,
            target.getActualTurretPositionTicks()
        );
        assertEquals(Turret.ControlMode.FIELD_FOLLOWING, target.getControlMode());
    }

    public void testSetTurretPosition() {
        target.setTurretAngle(Turret.CARDINAL_SOUTH);
        target.setControlMode(Turret.ControlMode.POSITION);
        target.writePeriodicOutputs();
        assertEquals(
            (double) Turret.ABS_TICKS_SOUTH,
            target.getActualTurretPositionTicks()
        );
        assertEquals(Turret.ControlMode.POSITION, target.getControlMode());
    }

    public void testSetTurretFieldFollowingRobotTwist() {
        RobotState state = new RobotState();
        //state.setHeadingRelativeToInitial(new Rotation2d(20));
        Turret turret = new Turret();
        turret.setTurretAngle(Turret.CARDINAL_SOUTH);
        turret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        turret.writePeriodicOutputs();
        assertEquals(
            (double) Turret.ABS_TICKS_SOUTH,
            turret.getActualTurretPositionTicks()
        );
        assertEquals(Turret.ControlMode.FIELD_FOLLOWING, turret.getControlMode());
    }

    public void testTurretAutoHome() {
        target.setControlMode(Turret.ControlMode.CAMERA_FOLLOWING);
    }
}
