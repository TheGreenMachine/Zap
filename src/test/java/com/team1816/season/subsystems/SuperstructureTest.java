package com.team1816.season.subsystems;

import com.team1816.TestUtil;
import com.team1816.lib.Injector;
import com.team1816.season.states.RobotState;
import com.team1816.season.states.Superstructure;
import junit.framework.TestCase;
import org.junit.Before;

public class SuperstructureTest extends TestCase {

    private final RobotState state;

    private Superstructure mSuperstructure;

    public SuperstructureTest() {
        TestUtil.SetupMockRobotFactory(null);
        state = Injector.get(RobotState.class);
    }

    @Before
    public void setUp() {
        mSuperstructure = Injector.get(Superstructure.class);
        state.resetPosition();
    }

    public void testSetStopped() {}

    public void testSetCollecting() {}

    public void testSetRevving() {}

    public void testSetFiring() {}

    public void testGetDistance() {}

    public void testGetPredictedDistance() {}
}
