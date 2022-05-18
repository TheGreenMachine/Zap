package com.team1816.season.subsystems;

import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import org.junit.Assert;
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
        state.reset();
    }

    public void checkCoordinates() {
        for (int i = 0; i < mDistanceManager.getCoordinates().size(); i++) {
            double expectedValue = mDistanceManager.getCoordinates().get(i)[1];
            double actualValue = mDistanceManager.getShooterOutput(
                mDistanceManager.getCoordinates().get(i)[0]
            );
            Assert.assertEquals(expectedValue, actualValue, 0.1);
        }
    }

    public void bucketTest(double lowerBound, double upperBound, double resolution) {
        mDistanceManager.calculateFloorFunctionSpline();
        checkCoordinates();
        for (double i = lowerBound; i <= upperBound; i += resolution) {
            double expectedValue = 0;
            for (int j = 1; j < mDistanceManager.getCoordinates().size(); j++) {
                if (i < mDistanceManager.getCoordinates().get(1)[0]) {
                    expectedValue = mDistanceManager.getCoordinates().get(0)[1];
                    break;
                }
                if (i < mDistanceManager.getCoordinates().get(j)[0]) {
                    expectedValue = mDistanceManager.getCoordinates().get(j - 1)[1];
                    break;
                }
                if (
                    i >=
                    mDistanceManager
                        .getCoordinates()
                        .get(mDistanceManager.getCoordinates().size() - 1)[0]
                ) {
                    expectedValue =
                        mDistanceManager
                            .getCoordinates()
                            .get(mDistanceManager.getCoordinates().size() - 1)[1];
                    break;
                }
            }
            Assert.assertEquals(expectedValue, mDistanceManager.getShooterOutput(i), 0.1);
        }
    }

    @Test
    public void runBucketTest() {
        double lowerBound = 95;
        double upperBound =
            mDistanceManager
                .getCoordinates()
                .get(mDistanceManager.getCoordinates().size() - 1)[0] *
            1.1;
        bucketTest(lowerBound, upperBound, 1.0);
    }

    @Test
    public void runLinearSplineTest() {} //needs to check coordinates and monotonically increasing conditions

    @Test
    public void runCubicSplineTest() {} //needs to check coordinates and monotonically increasing conditions
}
