package com.team1816.season.subsystems;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.when;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.hardware.components.motor.GhostMotorControllerEnhanced;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import com.team1816.season.SeasonModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;
import org.mockito.Spy;

// @RunWith(JUnit4.class)
public class TurretTest {

    private final RobotState state;
    private final RobotFactory mockFactory;
    private Turret mTurret;
    private double encTickSouth = 1980;
    private double encTick45 = encTickSouth + 512;
    private double encTick315 = encTickSouth - 512;
    private final double encPPR = 4096;

    @Spy
    private Constants constants;

    public TurretTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        when(mockFactory.getConstant(Turret.NAME, "absPosTicksSouth"))
            .thenReturn(encTickSouth);
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(encPPR);
        when(mockFactory.getConstant(Turret.NAME, "encPPR")).thenReturn(encPPR);
        Subsystem.factory = mockFactory;
        Injector injector = Guice.createInjector(new LibModule(), new SeasonModule());
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mTurret = new Turret();
        mTurret.zeroSensors();
        state.reset();
    }

    @Test
    public void fieldFollowingTest() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        assertEquals(0, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(encTickSouth, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void fieldFollowing45Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.field_to_vehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(45, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        // Turret should move CW
        assertEquals(encTick45, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void fieldFollowing315Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.field_to_vehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(-45));
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(315, state.vehicle_to_turret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret(), 0.1);
        // Turret should move CCW
        assertEquals(encTick315, mTurret.getActualTurretPositionTicks(), .01);
    }

    @Test
    public void fieldFollowingDoubleTest() {
        setupDoubleRotation();
        fieldFollowingTest();
    }

    @Test
    public void fieldFollowing45DoubleTest() {
        setupDoubleRotation();
        fieldFollowing45Test();
    }

    @Test
    public void fieldFollowing315DoubleTest() {
        setupDoubleRotation();
        fieldFollowing315Test();
    }

    @Test
    public void convertTurretDegreesToTicksTest() {
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(0), .01);
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(360), .01);
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(720), .01);
        assertEquals(encTick45, mTurret.convertTurretDegreesToTicks(45), .01);
        assertEquals(encTick315, mTurret.convertTurretDegreesToTicks(-45), .01);
    }

    @Test
    public void convertTurretTicksToDegrees() {
        assertEquals(0, mTurret.convertTurretTicksToDegrees(encTickSouth), .01);
        assertEquals(45, mTurret.convertTurretTicksToDegrees(encTick45), .01);
        assertEquals(315, mTurret.convertTurretTicksToDegrees(encTick315), .01);
    }

    @Test
    public void convertTurretTicksToDegreesDoubleRotation() {
        setupDoubleRotation();
        convertTurretTicksToDegrees();
    }

    @Test
    public void convertTurretDegreesToTicksDoubleRotationTest() {
        setupDoubleRotation();
        convertTurretDegreesToTicksTest();
    }

    private void setupDoubleRotation() {
        encTickSouth = 1980 * 2;
        encTick45 = encTickSouth + 1024;
        encTick315 = encTickSouth - 1024;
        when(mockFactory.getConstant(Turret.NAME, "absPosTicksSouth"))
            .thenReturn(encTickSouth);
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(encPPR * 2);
        mTurret = new Turret();
        mTurret.zeroSensors();
        state.reset();
    }

    @Test
    public void zeroSensors0Test() {
        zeroSensorsTest(0, 53248.0);
    }

    @Test
    public void zeroSensors2048Test() {
        zeroSensorsTest(2048, 53248.0);
    }

    @Test
    public void zeroSensors2048SingleRotationTest() {
        zeroSensorsTest(2048, 4096);
    }

    @Test
    public void zeroSensors4095Test() {
        zeroSensorsTest(4095, 53248.0);
    }

    public void zeroSensorsTest(int absInit, double turretPPR) {
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(turretPPR);
        when(mockFactory.getMotor(Turret.NAME, "turret"))
            .thenReturn(new GhostMotorControllerEnhanced(0, absInit));
        mTurret = new Turret();
        mTurret.zeroSensors();
        Assert.assertEquals(
            mTurret.TURRET_PPR / 2.0 - mTurret.TURRET_PPR == mTurret.TURRET_ABS_ENCODER_PPR
                ? 0
                : absInit,
            mTurret.getActualTurretPositionTicks(),
            1
        );
    }
}
