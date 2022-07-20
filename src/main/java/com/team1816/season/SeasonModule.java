package com.team1816.season;

import com.google.inject.AbstractModule;
import com.team1816.lib.SeasonFactory;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.controlboard.ControlBoardFactory;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.math.SwerveKinematics;
import com.team1816.lib.subsystems.Drive;
import com.team1816.lib.subsystems.SwerveDrive;
import com.team1816.lib.subsystems.TankDrive;
import com.team1816.season.auto.actions.*;
import com.team1816.season.controlboard.ControlBoard;
import com.team1816.season.states.Superstructure;
import com.team1816.season.subsystems.*;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(SeasonFactory.class);
        bind(IControlBoard.class).to(ControlBoard.class);
        requestStaticInjection(SwerveKinematics.class);
        requestStaticInjection(Drive.class);
        requestStaticInjection(Superstructure.class);
        requestStaticInjection(TankDrive.class);
        requestStaticInjection(SwerveDrive.class);
        requestStaticInjection(Camera.class);
        requestStaticInjection(Turret.class);
        requestStaticInjection(LedManager.class);
        requestStaticInjection(ControlBoardFactory.class);
        requestStaticInjection(TrajectoryAction.class);
        requestStaticInjection(TurretAction.class);
        requestStaticInjection(AutoAimAction.class);
        requestStaticInjection(DriveOpenLoopAction.class);
        requestStaticInjection(StopAction.class);
        requestStaticInjection(CollectAction.class);
        requestStaticInjection(RampUpShooterAction.class);
        requestStaticInjection(ShootAction.class);
    }
}
