package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.logger.*;
import com.team1678.frc2020.logger.LogStorage;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.LazySparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Intake extends Subsystem {
    private static double kIntakingVoltage = 12.0;
    private static double kIdleVoltage = 0;

    private static Intake mInstance;

    private Solenoid mDeploySolenoid;

    public enum WantedAction {
        NONE, INTAKE, RETRACT,
    }

    public enum State {
        IDLE, INTAKING, RETRACTING,
    }

    private State mState = State.IDLE;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mMaster;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;

        // OUTPUTS
        public double demand;
        public boolean deploy;
    }

    LogStorage<PeriodicIO> mStorage = null;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIntakeRollerId);
        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kDeploySolenoidId);
    }

    @Override
    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "intake.csv");
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putString("Intake State", mState.toString());
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // startLogging();
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    runStateMachine();

                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
                stop();
            }
        });
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine() {
        switch (mState) {
        case INTAKING:
                mPeriodicIO.demand = kIntakingVoltage;
                mPeriodicIO.deploy = true;
            break;
        case RETRACTING:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.deploy = false;
            break;
        case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
                mPeriodicIO.deploy = false;
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case RETRACT:
            mState = State.RETRACTING;
            break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        LogSend();
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mDeploySolenoid.set(mPeriodicIO.deploy);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public void LogSetup() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }
    public void LogSend() {
        ArrayList<Double> items = new ArrayList<Double>();
        items.add(Timer.getFPGATimestamp());
        items.add(mPeriodicIO.current);
        items.add(mPeriodicIO.demand);  
        mStorage.addData(items);
    }
}
