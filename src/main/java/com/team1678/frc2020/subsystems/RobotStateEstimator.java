package com.team1678.frc2020.subsystems;

import com.team1323.lib.geometry.UnwrappableRotation2d;
import com.team1678.frc2020.Kinematics;
import com.team1678.frc2020.RobotState;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;

public class RobotStateEstimator extends Subsystem {
	RobotState mRobotState = RobotState.getInstance();
	Swerve swerve;
	Turret turret;

    private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance() {
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator() {
	}

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
        }

        @Override
        public synchronized void onLoop(double timestamp) {
			mRobotState.addFieldToVehicleObservation(timestamp, swerve.getPose().wrap());
        }

        @Override
        public void onStop(double timestamp) {
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {

    }
}