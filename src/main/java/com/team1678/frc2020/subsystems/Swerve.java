package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.subsystems.Subsystem;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.SwerveKinematics;

public class Swerve extends Subsystem {
    private static Swerve mInstance = null;

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    } 

    private SwerveKinematics swerveKinematics = new SwerveKinematics();
    private SwerveDriveModule frontRight, frontLeft, rearRight, rearLeft;
    List<SwerveDriveModule> modules;

    public Swerve() {
        frontRight = new SwerveDriveModule(Constants.kFrontRightAngleSlot, Constants.kFrontRightDriveSlot, 0, Constants.kFrontRightEncoderStartingPose);
        frontLeft = new SwerveDriveModule(Constants.kFrontLeftAngleSlot, Constants.kFrontLeftDriveSlot, 1, Constants.kFrontLeftEncoderStartingPose);
        rearLeft = new SwerveDriveModule(Constants.kRearLeftAngleSlot, Constants.kRearLeftDriveSlot, 2, Constants.kRearLeftEncoderStartingPose);
        rearRight = new SwerveDriveModule(Constants.kRearRightAngleSlot, Constants.kRearRightDriveSlot, 3, Constants.kRearRightEncoderStartingPose);
        modules = Arrays.asList(frontRight, frontLeft, rearLeft, rearRight);
    }

    // x1 = strafe, y1 = speed, x2 = rotation 
    public void drive(double x1, double y1, double x2) {
        swerveKinematics.calculate(x1, y1, x2);

        for(int i=0; i < modules.size(); i++){
    			modules.get(i).setRotationOpenLoop(swerveKinematics.wheelAngles[i]);
    			modules.get(i).setDriveOpenLoop(swerveKinematics.wheelSpeeds[i]);
    	}
    }

	@Override
	public synchronized void readPeriodicInputs() {
		modules.forEach((m) -> m.readPeriodicInputs());
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
    }
    
    private final Loop mLoop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			synchronized(Swerve.this){}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Swerve.this){}
		}

		@Override
		public void onStop(double timestamp) {
			synchronized(Swerve.this){
				stop();
			}
		}
		
	};

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(mLoop);
    }
    
    public synchronized void disable() {
        modules.forEach((m) -> m.disable());
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void stop() {
        modules.forEach((m) -> m.stop());
    }

    @Override
    public void outputTelemetry() {
        modules.forEach((m) -> m.outputTelemetry());

    }
}