package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
//import com.team1678.lib.control.PIDController;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;

public class SwerveDriveModule extends Subsystem {
	private TalonFX mAngleMotor, mDriveMotor;
	int kMaxVoltage = 10;
    int moduleId;
    String name = "Module ";
	int encoderOffset;
	
	PeriodicIO periodicIO = new PeriodicIO();
	
	//PIDController mPidController;

    public static class PeriodicIO {
        // Inputs
        public int rotationPosition = 0;
        public int velocity = 0;
    
        // Outputs
        public ControlMode rotationControlMode = ControlMode.PercentOutput;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double rotationDemand;
		public double driveDemand;
	}
	
	//private PIDController pidController;

    public SwerveDriveModule(int angleMotorSlot, int driveMotorSlot, int moduleId, int encoderOffset) {
        name += (moduleId + " ");
        mAngleMotor = TalonFXFactory.createDefaultTalon(angleMotorSlot);
        mDriveMotor = TalonFXFactory.createDefaultTalon(driveMotorSlot);
        this.moduleId = moduleId;
		this.encoderOffset = encoderOffset;
		
		// pidController = new PIDController (1, 0, 0);
        // pidController.enableContinuousInput(-1, 1);
	}

    public void setRotationOpenLoop(double power) {
        periodicIO.rotationControlMode = ControlMode.PercentOutput;
        periodicIO.rotationDemand = power;
    }

    public void setDriveOpenLoop(double velocity) {
        periodicIO.driveControlMode = ControlMode.PercentOutput;
        periodicIO.driveDemand = velocity;
    }

    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(getRawAngle() - encUnitsToDegrees(encoderOffset));
    }

	public double getRawAngle() {
        return encUnitsToDegrees(periodicIO.rotationPosition);
	}
	
    public double encUnitsToInches(double encUnits) {
        return encUnits/Constants.kSwerveEncUnitsPerInch;
    }

	public double encVelocityToInchesPerSecond(double encUnitsPer100ms){
		return encUnitsToInches(encUnitsPer100ms) * 10;
	}
	
	public double encUnitsToDegrees(double encUnits){
		return encUnits/Constants.kSwerveDriveEncoderResolution*360.0;
	}

    @Override
    public synchronized void readPeriodicInputs() {
    	periodicIO.rotationPosition = mAngleMotor.getSelectedSensorPosition(0);
    	periodicIO.velocity = mDriveMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
		double setpoint = getRawAngle() * (kMaxVoltage * 0.5) + (kMaxVoltage * 0.5);
		if (setpoint < 0) {
			setpoint = kMaxVoltage + setpoint;
		}
		if (setpoint > kMaxVoltage) {
			setpoint = setpoint - kMaxVoltage;
		}
		// mPidController.setGoal(setpoint);
		periodicIO.rotationDemand = setpoint;
		
		mDriveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);
		mAngleMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
    }

    @Override
    public synchronized void stop() {
        setDriveOpenLoop(0.0);
    }

    public synchronized void disable() {
		setRotationOpenLoop(0.0);
		setDriveOpenLoop(0.0);
    }

	@Override
	public void zeroSensors() {}
	
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {}

    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
        SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
    }

}