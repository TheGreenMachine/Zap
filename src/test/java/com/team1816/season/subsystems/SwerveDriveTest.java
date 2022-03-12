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
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

// @RunWith(JUnit4.class)
public class SwerveDriveTest {

    private final RobotFactory mockFactory;
    private final RobotState state;
    private SwerveDrive mDrive;
    private double maxVel = 2.54; //  m per sec
    private double maxRotVel = 2 * Math.PI; // rad per sec;

    public SwerveDriveTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        Constants.kPathFollowingMaxVelMeters = maxVel;
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
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                robotAngle
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVel);
        return states;
    }

    @Test
    public void testFactoryMock() {
        assertEquals(maxVel, Constants.kPathFollowingMaxVelMeters, .01);
        assertEquals(maxRotVel, Constants.kMaxAngularSpeed, .01);
    }

    @Test
    public void testTeleopRotation() {
        mDrive.setTeleopInputs(0, 0, 1, false, false);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
        verifyStates(mDrive.getStates(), 0, maxVel, maxRotVel);
    }

    @Test
    public void testTeleopForward() {
        mDrive.setTeleopInputs(1, 0, 0, false, false);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
        verifyStates(mDrive.getStates(), maxVel, 0, 0);
    }

    @Test
    public void testTeleopStrafe() {
        mDrive.setTeleopInputs(0, 1, 0, false, false);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
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

        // We verify the returned value from getState to match the original value.
        // So even though we are percent output the getState is used for feedback
        // this needs to be real velocity values that are returned
        for (int i = 0; i < states.length; i++) {
            var actVel = states[i].speedMetersPerSecond;
            assertEquals(
                Math.abs(expected[i].speedMetersPerSecond),
                Math.abs(actVel),
                .01
            );
            var actRot = states[i].angle.getRadians();
            var expRot = expected[i].angle.getRadians();
            if (actRot >= 2 * Math.PI) actRot -= 2 * Math.PI;
            if (actVel < 0) expRot += Math.PI;
            assertEquals(expRot, actRot, .2);
        }
    }
}
