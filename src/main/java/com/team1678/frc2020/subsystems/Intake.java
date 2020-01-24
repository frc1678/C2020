package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.logger.*;
import com.team1678.frc2020.logger.LogStorage;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.drivers.LazySparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Intake extends Subsystem {
    public static double kIntakingVoltage = 12.0;
    public static double kOuttakingVoltage = -12.0;
    public static double kIdleVoltage = 0;

    public static Intake mInstanceIntake;

    public enum WantedAction {
        NONE, INTAKE, OUTTAKE,
    }

    public enum State {
        IDLE, INTAKING, OUTTAKING,
    }

    private State mState = State.IDLE;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();

    private final LazySparkMax mMaster;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;

        // OUTPUTS
        public double demand;
    }
    LogStorage mStorage = null;


    private Intake() {
        mMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kIntakeRollerID);
    }
    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "intake.csv");
    }

    public synchronized static Intake getInstance() {
        if (mInstanceIntake == null) {
            mInstanceIntake = new Intake();
        }
        return mInstanceIntake;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
    }

    @Override
    public void stop() {
        mMaster.set(0);
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
            break;
        case OUTTAKING:
                mPeriodicIO.demand = kOuttakingVoltage;
            break;
        case IDLE:
                mPeriodicIO.demand = kIdleVoltage;
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
        case OUTTAKE:
            mState = State.OUTTAKING;
            break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        LogSend();
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(mPeriodicIO.demand / 12.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public void LogSetup() {
        mStorage = new LogStorage();
        ArrayList<String> columnNames = new ArrayList<String>();
        columnNames.add("timestamp");
        columnNames.add("current");
        columnNames.add("demand");
        mStorage.setHeaders(columnNames);
    }
    public void LogSend() {
        ArrayList<Double> items = new ArrayList<Double>();
        items.add(Timer.getFPGATimestamp());
        items.add(mPeriodicIO.current);
        items.add(mPeriodicIO.demand);  
        mStorage.addData(items);
    }
 }
