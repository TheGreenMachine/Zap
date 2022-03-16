package com.team1816.season;

import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Provider;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.RobotFactory;
import com.team1816.season.subsystems.Drive;
import com.team1816.season.subsystems.SwerveDrive;
import com.team1816.season.subsystems.TankDrive;

@Singleton
public class DriveProvider implements Provider<Drive> {
    private final RobotFactory factory = Robot.getFactory();
    private static Drive mDrive;

    @Inject
    Injector injector;

    @Override
    public Drive get() {
        if (mDrive == null) {
            boolean isSwerve = factory.getConstant(Drive.NAME, "isSwerve") == 1;
            if (isSwerve) {
                mDrive = injector.getInstance(SwerveDrive.class);
            } else {
                mDrive = injector.getInstance(TankDrive.class);
            }
            System.out.println("Created " + mDrive.getClass().getSimpleName());
        }
        return mDrive;
    }
}
