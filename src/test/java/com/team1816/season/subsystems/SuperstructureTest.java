package com.team1816.season.subsystems;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.LibModule;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.SeasonModule;
import com.team1816.season.states.Superstructure;
import junit.framework.TestCase;
import org.junit.Before;
import org.mockito.Mockito;
import org.mockito.Spy;

public class SuperstructureTest extends TestCase {

    private final RobotState state;
    private final RobotFactory mockFactory;
    private Superstructure mSuperstructure;

    @Spy
    private Constants constants;

    public SuperstructureTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        //        when(mockFactory.getConstant(Turret.NAME, "encPPR")).thenReturn(encPPR);
        Subsystem.factory = mockFactory;
        Injector injector = Guice.createInjector(new LibModule(), new SeasonModule());
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mSuperstructure = new Superstructure();
        state.reset();
    }

    public void testSetStopped() {}

    public void testSetCollecting() {}

    public void testSetRevving() {}

    public void testSetFiring() {}

    public void testGetDistance() {}

    public void testGetPredictedDistance() {}
}
