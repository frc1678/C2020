package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mMaster;
    private final TalonFX mSlave;

    private boolean mSpunUp = false;
    private boolean mRunningManual = false;

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
        SmartDashboard.putNumber("Shooter Velocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Shooter Current", mPeriodicIO.current);
        SmartDashboard.putNumber("Shooter Goal", mPeriodicIO.demand);
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
        mPeriodicIO.demand = percentage;
        mRunningManual = true;
    }

    public synchronized double getVoltage() {
        return mPeriodicIO.demand;
    }

    public synchronized double getShooterRPM() {
        return mMaster.getSelectedSensorVelocity();
    }

    public synchronized double getVelocity() {
        return mPeriodicIO.velocity;
    }

    public synchronized boolean spunUp() {
        return (Math.abs(mPeriodicIO.velocity - mPeriodicIO.demand) < kShooterTolerance);
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.demand = velocity;
        mRunningManual = false;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity() * kVelocityConversion;
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.temperature = mMaster.getTemperature();
    }

    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.demand / kVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
        }
    }
    @Override
    public synchronized boolean checkSystem() {
        return true;
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double velocity;
        public double current;
        public double temperature;
        public double voltage;
        
        //OUTPUTS
        public double demand;
    }
}
