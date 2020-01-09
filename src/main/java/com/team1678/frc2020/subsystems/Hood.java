package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.team254.drivers.LazyTalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Encoder;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class Hood extends Subsystem {



    public static double riseVoltage;
    public static double dropVoltage;
    public static double holdVoltage;

    public boolean aimed = false;

    private static Hood mInstance;

    public enum WantedAction {
        NONE, AIMED, UP, DOWN,
    }

    private enum State {
        IDLE, UPING, DOWNING, AIMED, MAX, MIN,
    }

    private State mState = State.IDLE;

    // private variables
    private boolean mRunningManual = false;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    // Motors, sensors, and the solenoids
    private final LazySparkMax mMaster;
    //private final Encoder mEncoder;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
        sparkMax.setInverted(!left);
        sparkMax.enableVoltageCompensation(12.0);
        sparkMax.setClosedLoopRampRate(Constants.kDriveVoltageRampRate);
    }

    private Hood() {
        mMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kLeftDriveMasterId);
        configureSpark(mMaster, true, true);
    }

    public synchronized static Hood getInstance() {
     if (mInstance == null) {
            mInstance = new Hood();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {

         SmartDashboard.putNumber("MotorSetpoint", mPeriodicIO.demand);
         SmartDashboard.putBoolean("Aimed", mPeriodicIO.isReady);

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
    public void registerEnabledLoops(final ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(final double timestamp) {
                mRunningManual = false;
                mState =  State.IDLE;
                startLogging();
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (Hood.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(final double timestamp) {
                mRunningManual = false;
                mState = State.IDLE;
                stopLogging();
            }
        });
    }


    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            mPeriodicIO.demand = 0;
            mPeriodicIO.isReady = false;
            mPeriodicIO.maxed = false;
        case UPING:
            mPeriodicIO.demand = riseVoltage;
            //mMaster.
            mPeriodicIO.isReady = false;
            mPeriodicIO.maxed = false;
        case DOWNING:
            mPeriodicIO.demand = dropVoltage;
            mPeriodicIO.isReady = false;
            mPeriodicIO.maxed = false;
        case AIMED:
            mPeriodicIO.demand = holdVoltage;
            mPeriodicIO.isReady = true;
            mPeriodicIO.maxed = false;
        case MAX:
            mPeriodicIO.demand = 0;
            mPeriodicIO.isReady = false;
            mPeriodicIO.maxed = true;
        case MIN:
            mPeriodicIO.demand = 0;
            mPeriodicIO.isReady = false;
            mPeriodicIO.maxed = true;
        }

    }


    public void setState(final WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            if (mState == State.AIMED){
                mState = State.IDLE;
            }
        case AIMED:
            if (mState == State.IDLE && aimed == true){
                mState = State.AIMED;
            }
        case UP:
            if (mState == State.UPING){
                mState = State.IDLE;
            }
        case DOWN:
            if (mState == State.DOWNING){
                mState = State.IDLE;
            }
        }
    }

    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.demand = percentage;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // Update anything you want to read from sensors or actuators, these are usually in your inputs under periodicIO

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.currentPos > mPeriodicIO.wantedPos){
            setState(WantedAction.DOWN);
        } else {
            setState(WantedAction.UP);
        }
        // Use .set on each of your actuators to whatever output you have been setting from periodicIO. This is also a good place to add limits to your code. 
    
    }

    @Override
    public boolean checkSystem() {
    // Use CheckMotor function on any motor you have that uses PID, else just return true in this block
    return true;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/HOOD-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        // public double motorTempature;
        public double currentPos;
        public double wantedPos;
  

        // OUTPUTS
        public double demand;
        public boolean isReady;
        public boolean maxed;
    }




}