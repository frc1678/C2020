package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import com.team254.lib.drivers.BaseTalonChecker;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Intake extends Subsystem {
    public static double kintakingVoltage = 12.0;
    public static double kouttakingVoltage = -12.0;
    public static double kidleVoltage = 0;

    public static Intake mInstanceIntake;

    public enum WantedAction {
        IDLE, INTAKING, OUTTAKING,
    }
    public enum State {
        IDLE, INTAKING, OUTTAKING,
    }
    private State mState = State.IDLE;

    private boolean mRunningManual = false;

    private static PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private final TalonFX mMaster;

    private ReflectingCSVWriter<PeriodicIO> mCsvWriter = null;

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double current;
        
        //OUTPUTS
        public static double demand;
    }

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(6); // Constants.kIntakeRollerId); // check constants for ID and name
        mMaster.configForwardSoftLimitEnable(false); 

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12);
        mMaster.enableVoltageCompensation(true);
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

        if (mCsvWriter != null) {
            mCsvWriter.write();
        }
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
                stopLogging();

            }
        });
    }

    public void runStateMachine(boolean modifyingOutputs) {
        switch (mState) {
        case INTAKING:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kintakingVoltage;
            }
            break;
        case OUTTAKING:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kouttakingVoltage;
            }
            break;
        case IDLE:
            if (modifyingOutputs) {
                mPeriodicIO.demand = kidleVoltage;
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
        case IDLE:
            mState = State.IDLE;
            break;
        case INTAKING:
            mState = State.INTAKING;
            break;
        case OUTTAKING:
            mState = State.OUTTAKING;
            break;
        }

    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mCsvWriter != null) {
            mCsvWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, PeriodicIO.demand / 12.0);
    }
    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = -4824415636161505593L;

            {
                add(new MotorChecker.MotorConfig<>("intake_motor", mMaster));
            }
        }, new BaseTalonChecker.CheckerConfig() {
            {
                mCurrentFloor = 2;
                mRPMFloor = 1500;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 250;
                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
            }
        });
    }
    public synchronized void startLogging() {
        if (mCsvWriter == null) {
            mCsvWriter = new ReflectingCSVWriter<>("/home/lvuser/INTAKE-LOGS.csv", PeriodicIO.class); 
        }
    }
    public synchronized void stopLogging() {
        if (mCsvWriter != null) {
            mCsvWriter.flush();
            mCsvWriter = null;
        }
    }
 }
