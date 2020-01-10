package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    private boolean spun_up;

    private static double kVelocityConversion = 600.0 / 2048.0;
    private static double kShooterTolerance = 600.0;

    private Shooter() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterFlywheelID);
        mSlave = TalonFXFactory.createDefaultTalon(Constants.kSlaveFlywheelID);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false); //TODO: check value
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        
        mMaster.config_kP(0, Constants.kShooterP);
        mMaster.config_kI(0, Constants.kShooterI);
        mMaster.config_kD(0, Constants.kShooterD);
        mSlave.follow(mMaster);
        mSlave.setInverted(false); //TODO: check value
    }
    public synchronized static Shooter mInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("ShooterVelocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("ShooterCurrent", mPeriodicIO.current);
        SmartDashboard.putNumber("ShooterGoalVelocity", mPeriodicIO.goal_velocity);
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }
    @Override
    public void stop() {
        setOpenLoop(0);
    }
    @Override
    public void zeroSensors() {
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override
            public void onLoop(double timestamp) {
            }
            @Override
            public void onStop(double timestamp) {
            }
        });
    }
    public synchronized void setOpenLoop(double percentage) {
        mMaster.set(TalonFXControlMode.PercentOutput, percentage);
    }
    public synchronized double getVoltage() {
        return mPeriodicIO.demand;
    }
    public synchronized boolean spunUp() {
        if (Math.abs(mPeriodicIO.velocity - mPeriodicIO.goal_velocity) < kShooterTolerance) {
            spun_up = true;
        }  else {
            spun_up = false;
        }        
        return spun_up;
    }
    public synchronized void setGoal(double velocity) {
        mPeriodicIO.goal_velocity = velocity;
      
    }
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity() * kVelocityConversion;
        mPeriodicIO.demand = mMaster.getMotorOutputVoltage();
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.temperature = mMaster.getTemperature();
    }
    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.Velocity, mPeriodicIO.goal_velocity / kVelocityConversion);
    }
    @Override
    public synchronized boolean checkSystem() {
        return true;
    }
    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double velocity;
        public double demand;
        public double current;
        public double temperature;
        
        //OUTPUTS
        public double goal_velocity;
    }
}