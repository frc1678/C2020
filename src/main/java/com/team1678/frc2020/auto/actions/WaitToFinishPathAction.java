package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Swerve;

import edu.wpi.first.wpilibj.Timer;

public class WaitToFinishPathAction implements Action{
	Swerve swerve;
	double timeout;
	double startTime;
	
	public WaitToFinishPathAction(){
		swerve = Swerve.getInstance();
		timeout = 15.0;
	}
	
	public WaitToFinishPathAction(double timeout){
		swerve = Swerve.getInstance();
		this.timeout = timeout;
	}
	
	@Override
	public boolean isFinished(){
		return swerve.hasFinishedPath() || ((Timer.getFPGATimestamp() - startTime) > timeout);
	}
	
	@Override
	public void start(){
		startTime = Timer.getFPGATimestamp();
	}
	
	@Override
	public void update(){
	}
	
	@Override
	public void done(){
	}
}
