package com.team1816.season.subsystems;

import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import junit.framework.TestCase;
import org.junit.Before;
import org.mockito.Mockito;
import org.mockito.Spy;

public class SuperstructureTest extends TestCase {

    private final RobotState state;
    private final RobotFactory mockFactory;
    private final Drive.Factory mDriveFactory;
    private Superstructure mSuperstructure;

    @Spy
    private Constants constants;

    private final Injector injector;

    public SuperstructureTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        mDriveFactory = Mockito.mock(Drive.Factory.class);
        Subsystem.factory = mockFactory;
        injector =
            Guice.createInjector(
                new AbstractModule() {
                    @Override
                    protected void configure() {
                        bind(Drive.Factory.class).toInstance(mDriveFactory);
                        bind(Drive.class).toInstance(Mockito.mock(Drive.class));
                        requestStaticInjection(Superstructure.class);
                    }
                }
            );
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mSuperstructure = new Superstructure();
        state.resetPosition();
    }

    public void testSetStopped() {}

    public void testSetCollecting() {}

    public void testSetRevving() {}

    public void testSetFiring() {}

    public void testGetDistance() {}

    public void testGetPredictedDistance() {}
}
