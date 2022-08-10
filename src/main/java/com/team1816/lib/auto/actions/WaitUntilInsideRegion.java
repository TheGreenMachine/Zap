package com.team1816.lib.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class WaitUntilInsideRegion implements Action {

    private static RobotState mRobotState;

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;
    private String name = "";

    public WaitUntilInsideRegion(
        Translation2d bottomLeft,
        Translation2d topRight,
        String name
    ) {
        this.name = name;
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
        mRobotState = Injector.get(RobotState.class);
    }

    @Override
    public boolean isFinished() {
        Pose2d position = mRobotState.getLatestFieldToVehicle();
        var x = Units.metersToInches(position.getX());
        var y = Units.metersToInches(position.getY());
        return (
            x > mBottomLeft.getX() &&
            x < mTopRight.getX() &&
            y > mBottomLeft.getY() &&
            y < mTopRight.getY()
        );
    }

    @Override
    public void update() {}

    @Override
    public void done() {
        System.out.println("INSIDE DESIRED REGION: " + name);
    }

    @Override
    public void start() {}
}
