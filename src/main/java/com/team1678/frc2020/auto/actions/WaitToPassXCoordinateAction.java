package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Swerve;

public class WaitToPassXCoordinateAction implements Action{
	double startingXCoordinate;
	double targetXCoordinate;
	Swerve swerve;
	
	public WaitToPassXCoordinateAction(double x){
		targetXCoordinate = x;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate) !=
				Math.signum(swerve.getPose().getTranslation().x() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = swerve.getPose().getTranslation().x();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}

}
