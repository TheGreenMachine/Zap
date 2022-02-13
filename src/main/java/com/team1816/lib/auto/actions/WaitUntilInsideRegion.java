package com.team1816.lib.auto.actions;

import com.google.inject.Inject;
import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class WaitUntilInsideRegion implements Action {

    @Inject
    private static RobotState mRobotState;

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
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
    public void done() {}

    @Override
    public void start() {}
}
