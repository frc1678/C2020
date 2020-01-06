package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.loops.Looper;
import com.team1678.frc2020.subsystems.Drive.PeriodicIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.CANifier;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonFX;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.security.Timestamp;
import java.util.ArrayList;

public class Intake extends Subsystem {
    public static double intakingVoltage = 12.0;
    public static double outakingVoltage = -12.0;
    public static double idleVoltage = 0;

    public static double kIntakeRollerId; 

    public static Intake mInstanceIntake; 

    public enum WantedAction {
        IDLE, INTAKING, OUTAKING,
    }
    public enum State {
        IDLE, INTAKING, OUTAKING, 
    }
    private State mState = State.IDLE;

    private boolean mRunningManual = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private final TalonFX mMaster;

    private ReflectingCSVWriter<PeriodicIO> mCsvWriter = null;

    public static class PeriodicIO {
        //INPUTS
        public double timestamp;
        public double current;
        
        //OUTPUTS
        public double demand;
    }

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIntakeRollerId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        // TODO (eithne) add mMaster config voltage saturation, forwardSoft, reverseSoft limit
    }

    public synchronized static Intake getInstIntake() {
        if (mInstanceIntake == null) {
            mInstanceIntake = new Intake();
        }
        return mInstanceIntake;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);

        if (mCsvWriter != null) {
            mCsvWriter.Writer;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

   /* @Override
    public void registerEnabledLoops (ILooper enabledLooper) {
        enabledLooper.register(new Loop {
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
    } logger stuff talk to jishnu*/

    public void runStateMachine(boolean modifyOutputs) {
        switch(mState){
        case INTAKING:
            if () {
                break;
            }
            if (modifyOutputs) {

            }
            break;
        case OUTAKING:
            if () {

            } else if () {

            }
        case IDLE:
            if () {
                break;
            }
        }

    }
    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.demand = percentage;
    }
    public void getVoltage() {
        return mPeriodicIO.demand;
    }
    public void setState () {

    }
    @Override
    public synchronized void readPeriodicInputs() {
        if (mCsvWriter != null) {
            mCsvWriter.add(mPeriodicIO);
        }
    }
    @Override
    public boolean checkSystem() {

    }
    public synchronized void startLogging() {
        if (mCsvWriter == null) {
            //mCsvWriter = new ReflectingCSVWriter<>("/home/, PeriodicIO.class)
        }
    }
    public synchronized void stopLogging() {
        if (mCsvWriter != null) {
            mCsvWriter.flush();
            mCsvWriter = null;
        }
    }
 }
