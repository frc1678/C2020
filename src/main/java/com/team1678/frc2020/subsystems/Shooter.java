package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final TalonFX mTrigger;

    private boolean mRunningManual = false;

    private static double kFlywheelVelocityConversion = 600.0 / 2048.0;
    private static double kTriggerVelocityConversion = 600.0 / 2048.0;

    private static double kShooterTolerance = 600.0;
    private Solenoid popoutSolenoid;

    private Shooter() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kMasterFlywheelID);
        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kSlaveFlywheelID, Constants.kMasterFlywheelID);
        mTrigger = TalonFXFactory.createDefaultTalon(Constants.kTriggerWheelID);
        popoutSolenoid = Constants.makeSolenoidForId(Constants.kPopoutSolenoidId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false); //TODO: check value
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        
        mMaster.config_kP(0, 0.1, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, 0, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, 0.0, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kShooterF, Constants.kLongCANTimeoutMs);
        mMaster.selectProfileSlot(0, 0);

        mMaster.configClosedloopRamp(0.5);
        SupplyCurrentLimitConfiguration curr_limit = new SupplyCurrentLimitConfiguration(true, 40, 100, 0.5);
        mMaster.configSupplyCurrentLimit(curr_limit);
        mSlave.follow(mMaster);
        mSlave.setInverted(true); //TODO: check value

        mMaster.set(ControlMode.PercentOutput, 0);

        mTrigger.config_kP(0, Constants.kShooterP, Constants.kLongCANTimeoutMs);
        mTrigger.config_kI(0, 0, Constants.kLongCANTimeoutMs);
        mTrigger.config_kD(0, Constants.kShooterD, Constants.kLongCANTimeoutMs);
        mTrigger.config_kF(0, Constants.kShooterF, Constants.kLongCANTimeoutMs);

        mTrigger.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mTrigger.set(ControlMode.PercentOutput, 0);
        mTrigger.setInverted(false); //TODO: check value
        mTrigger.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mTrigger.enableVoltageCompensation(false);
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
        SmartDashboard.putNumber("Flywheel Velocity", mPeriodicIO.flywheel_velocity);
        SmartDashboard.putNumber("Flywheel Current", mPeriodicIO.flywheel_current);
        SmartDashboard.putNumber("Flywheel Goal", mPeriodicIO.flywheel_demand);
        SmartDashboard.putNumber("Flywheel Voltage", mPeriodicIO.flywheel_voltage);
        SmartDashboard.putNumber("Flywheel Temperature", mPeriodicIO.flywheel_temperature);

        SmartDashboard.putBoolean("Popout", mPeriodicIO.popout);
        SmartDashboard.putBoolean("Real Popout", popoutSolenoid.get());
        SmartDashboard.putNumber("Trigger Velocity", mPeriodicIO.trigger_velocity);
        SmartDashboard.putNumber("Trigger Current", mPeriodicIO.trigger_current);
        SmartDashboard.putNumber("Trigger Goal", mPeriodicIO.trigger_demand);
        SmartDashboard.putNumber("Trigger Temperature", mPeriodicIO.trigger_temperature);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0, 0, false);
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

    public synchronized void setOpenLoop(double flywheel, double trigger, boolean popout) {
        mPeriodicIO.flywheel_demand = flywheel;
        mPeriodicIO.trigger_demand = trigger;
        mRunningManual = true;
        mPeriodicIO.popout = popout;
    }

    public synchronized double getVoltage() {
        return mPeriodicIO.flywheel_demand;
    }

    public synchronized double getShooterRPM() {
        return mMaster.getSelectedSensorVelocity();
    }

    public synchronized double getTriggerRPM() {
        return mTrigger.getSelectedSensorVelocity();
    }

    public synchronized double getVelocity() {
        return mPeriodicIO.flywheel_velocity;
    }

    public synchronized boolean spunUp() {
        return (Util.epsilonEquals(mPeriodicIO.flywheel_demand, mPeriodicIO.flywheel_velocity, kShooterTolerance) &&
                (Util.epsilonEquals(mPeriodicIO.trigger_demand, mPeriodicIO.trigger_velocity, kShooterTolerance)));
    }

    public synchronized void setVelocity(double velocity) {
        mPeriodicIO.flywheel_demand = velocity;
        if (velocity > 0) {
            mPeriodicIO.popout = true;
            mPeriodicIO.trigger_demand = Constants.kTriggerRPM;
        } else {
            mPeriodicIO.popout = false;
            mPeriodicIO.trigger_demand = 0;
        }
        mRunningManual = false;
    }
    public synchronized void setPopout(boolean popout) {
        mPeriodicIO.popout = popout;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        
        mPeriodicIO.flywheel_velocity = mMaster.getSelectedSensorVelocity() * kFlywheelVelocityConversion;
        mPeriodicIO.flywheel_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.flywheel_current = mMaster.getStatorCurrent();
        mPeriodicIO.flywheel_temperature = mMaster.getTemperature();

        mPeriodicIO.trigger_velocity = mTrigger.getSelectedSensorVelocity() * kFlywheelVelocityConversion;
        mPeriodicIO.trigger_voltage = mTrigger.getMotorOutputVoltage();
        mPeriodicIO.trigger_current = mTrigger.getStatorCurrent();
        mPeriodicIO.trigger_temperature = mTrigger.getTemperature();
    }

    @Override
    public void writePeriodicOutputs() {
        if (!mRunningManual) {
            mMaster.set(ControlMode.Velocity, mPeriodicIO.flywheel_demand / kFlywheelVelocityConversion);
            mTrigger.set(ControlMode.Velocity, mPeriodicIO.trigger_demand / kTriggerVelocityConversion);
        } else {
            mMaster.set(ControlMode.PercentOutput, 0);
            mTrigger.set(ControlMode.PercentOutput, mPeriodicIO.trigger_demand);
        }
        popoutSolenoid.set(mPeriodicIO.popout);
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
        
        public double flywheel_velocity;
        public double flywheel_voltage;
        public double flywheel_current;
        public double flywheel_temperature;

        public double trigger_velocity;
        public double trigger_current;
        public double trigger_voltage;
        public double trigger_temperature;
        
        //OUTPUTS
        public double flywheel_demand;
        public double trigger_demand;
        public boolean popout;
    }
}
