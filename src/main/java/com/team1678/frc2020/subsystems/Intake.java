package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import com.team254.lib.drivers.SparkMaxFactory;

import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

    private boolean mRunningManual = false;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private final LazySparkMax mMaster;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double current;
        
        //OUTPUTS
        public double demand;
    }

    private Intake() {
        mMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kIntakeRollerID);
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

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
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
                mRunningManual = false;
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.IDLE;
                stop();
                stopLogging();
            }
        });
    }

    public void runStateMachine(boolean modifyingOutputs) {
        switch (mState) {
        case INTAKING:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kIntakingVoltage;
            }
            break;
        case OUTTAKING:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kOuttakingVoltage;
            }
            break;
        case IDLE:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kIdleVoltage;
            }
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.demand = percentage;
    }

    public double getVoltage() {
        return mPeriodicIO.demand;
    }

    public void setState(WantedAction wanted_state) {
        mRunningManual = false;
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
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(mPeriodicIO.demand / 12.0);
    }
    @Override
    public boolean checkSystem() {
        return true;
    }
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/INTAKE-LOGS.csv", PeriodicIO.class); 
        }
    }
    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
 }
