package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team1323.lib.trajectory.Trajectory;
import com.team1323.lib.trajectory.timing.TimedState;

public class SetTrajectoryAction extends RunOnceAction{
	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    double goalHeading;
    double rotationScalar;
	Swerve mSwerve;
	
	public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		mSwerve = Swerve.getInstance();
	}
	
	@Override
	public synchronized void runOnce(){
		mSwerve.setTrajectory(trajectory, goalHeading, rotationScalar);
	}
}
