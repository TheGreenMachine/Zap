package com.team1816.season;

import com.google.inject.AbstractModule;
import com.team1816.lib.auto.AutoModeExecutor;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.controlboard.IButtonControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.controlboard.IDriveControlBoard;
import com.team1816.lib.subsystems.Infrastructure;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.RampUpShooterAction;
import com.team1816.season.auto.actions.ShootAction;
import com.team1816.season.auto.actions.TurretAction;
import com.team1816.season.controlboard.ControlBoard;
import com.team1816.season.controlboard.GamepadButtonControlBoard;
import com.team1816.season.controlboard.GamepadDriveControlBoard;
import com.team1816.season.paths.TrajectorySet;
import com.team1816.season.subsystems.*;

public class SeasonModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(SeasonFactory.class);
        bind(IControlBoard.class).to(ControlBoard.class);
        bind(IDriveControlBoard.class).to(GamepadDriveControlBoard.class);
        bind(IButtonControlBoard.class).to(GamepadButtonControlBoard.class);
        requestStaticInjection(SwerveKinematics.class);
        requestStaticInjection(Drive.class);
        requestStaticInjection(Orchestrator.class);
        requestStaticInjection(TankDrive.class);
        requestStaticInjection(SwerveDrive.class);
        requestStaticInjection(Shooter.class);
        requestStaticInjection(Infrastructure.class);
        requestStaticInjection(Camera.class);
        requestStaticInjection(Turret.class);
        requestStaticInjection(AutoModeSelector.class);
        requestStaticInjection(AutoModeExecutor.class);
        requestStaticInjection(DistanceManager.class);
        requestStaticInjection(TrajectorySet.class);
        requestStaticInjection(TrajectoryAction.class);
        requestStaticInjection(TurretAction.class);
        requestStaticInjection(CollectAction.class);
        requestStaticInjection(RampUpShooterAction.class);
        requestStaticInjection(ShootAction.class);
    }
}
