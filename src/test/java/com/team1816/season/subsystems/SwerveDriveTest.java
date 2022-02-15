package com.team1816.season.subsystems;

import static org.junit.Assert.assertEquals;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.RobotState;
import com.team1816.season.SeasonModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

// @RunWith(JUnit4.class)
public class SwerveDriveTest {

    private final RobotFactory mockFactory;
    private final RobotState state;
    private SwerveDrive mDrive;
    private double maxVel = 100; //  in per sec to match yaml we need to convert to metric in constants class;
    private double maxRotVel = 2 * Math.PI; // rad per sec;

    public SwerveDriveTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        Constants.kMaxVel = maxVel;
        Constants.kMaxAngularSpeed = maxRotVel;
        Subsystem.factory = mockFactory;
        Injector injector = Guice.createInjector(new LibModule(), new SeasonModule());
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mDrive = new SwerveDrive();
        mDrive.zeroSensors();
        state.reset();
    }

    private SwerveModuleState[] getExpectedState(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        Rotation2d robotAngle
    ) {
        // TODO remove conversion when constants class is converted to metric
        var states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                Units.inchesToMeters(vxMetersPerSecond),
                Units.inchesToMeters(vyMetersPerSecond),
                omegaRadiansPerSecond,
                robotAngle
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Units.inchesToMeters(maxVel));
        return states;
    }

    @Test
    public void testFactoryMock() {
        assertEquals(maxVel, Constants.kMaxVel, .01);
        assertEquals(maxRotVel, Constants.kMaxAngularSpeed, .01);
    }

    @Test
    public void testTeleopForwardWithRotation() {
        mDrive.setTeleopInputs(1, 0, 1, false, false);
        mDrive.writePeriodicOutputs();
        mDrive.readPeriodicInputs();
        verifyStates(mDrive.getStates(), maxVel, 0, maxRotVel);
    }

    @Test
    public void testTeleopStrafeWithRotation() {
        mDrive.setTeleopInputs(0, 1, 1, false, false);
        mDrive.writePeriodicOutputs();
        mDrive.readPeriodicInputs();
        verifyStates(mDrive.getStates(), 0, maxVel, maxRotVel);
    }

    @Test
    public void testTeleopForward() {
        mDrive.setTeleopInputs(1, 0, 0, false, false);
        mDrive.writePeriodicOutputs();
        mDrive.readPeriodicInputs();
        verifyStates(mDrive.getStates(), maxVel, 0, 0);
    }

    @Test
    public void testTeleopStrafe() {
        mDrive.setTeleopInputs(0, 1, 0, false, false);
        mDrive.writePeriodicOutputs();
        mDrive.readPeriodicInputs();
        verifyStates(mDrive.getStates(), 0, maxVel, maxRotVel);
    }

    public void verifyStates(
        SwerveModuleState[] states,
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond
    ) {
        var expected = getExpectedState(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond,
            Constants.EmptyRotation
        );
        for (int i = 0; i < states.length; i++) {
            assertEquals(
                expected[i].speedMetersPerSecond,
                states[i].speedMetersPerSecond,
                .01
            );
            assertEquals(
                expected[i].angle.getRadians(),
                states[i].angle.getRadians(),
                .01
            );
        }
    }
}
