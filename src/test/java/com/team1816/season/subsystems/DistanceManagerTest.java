package com.team1816.season.subsystems;

import static org.junit.Assert.assertEquals;

import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

// @RunWith(JUnit4.class)
public class DistanceManagerTest {

    private final RobotState state;
    private final RobotFactory mockFactory;
    private DistanceManager mDistanceManager;

    public DistanceManagerTest() {
        mockFactory = Mockito.spy(RobotFactory.class);
        Subsystem.factory = mockFactory;
        Injector injector = Guice.createInjector(
            new AbstractModule() {
                @Override
                protected void configure() {
                    requestStaticInjection(DistanceManager.class);
                    requestStaticInjection(Shooter.class);
                }
            }
        );
        state = injector.getInstance(RobotState.class);
    }

    @Before
    public void setUp() {
        mDistanceManager = new DistanceManager();
        mDistanceManager.setCoordinates(mDistanceManager.getShooterMap());
        state.reset();
    }

    public void bucketTest(double lowerBound, double upperBound, double resolution) {
        mDistanceManager.calculateFloorFunctionSpline();
        for (double i = lowerBound; i <= upperBound; i += resolution) {
            double expectedValue = 0;
            for (int j = 0; j < mDistanceManager.getCoordinates().size(); j++) {
                if (i < mDistanceManager.getCoordinates().get(0)[0]) {
                    expectedValue = mDistanceManager.getCoordinates().get(0)[1];
                    break;
                } else if (i > mDistanceManager.getCoordinates().get(j)[0]) {
                    if (j - 1 >= 0) expectedValue =
                        mDistanceManager
                            .getCoordinates()
                            .get(j - 1)[1]; else expectedValue =
                        mDistanceManager.getCoordinates().get(0)[1];
                    break;
                }
            }
            assertEquals(expectedValue, mDistanceManager.getShooterOutput(i), 0.1);
        }
    }

    @Test
    public void runBucketTest() {
        double lowerBound = 97;
        double upperBound =
            mDistanceManager
                .getCoordinates()
                .get(mDistanceManager.getCoordinates().size() - 1)[0] *
            1.1;
        bucketTest(lowerBound, upperBound, 1.0);
    }

    @Test
    public void runLinearSplineTest() {}

    @Test
    public void runCubicSplineTest() {}
}
