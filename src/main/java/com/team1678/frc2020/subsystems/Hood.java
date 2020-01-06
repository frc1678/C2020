package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.drivers.LazyTalonSRX;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.SetpointGenerator;
import com.team254.lib.motion.SetpointGenerator.Setpoint;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class Hood extends Subsystem {

    //public static double kVoltage;

    private static Hood mInstance;

    public enum WantedAction {
        NONE, UP, DOWN,
    }

    private enum State {
        IDLE, IN_PROGRESS, UP,
    }

    private State mState = State.IDLE;

    // private variables
    private boolean mRunningManual = false;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    // Motors, sensors, and the solenoids
    private final LazyTalonSRX mMaster;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Hood() {
        mMaster = new LazyTalonSRX(Ports.HOOD)

        mProxy = new DigitalInput(Ports.HOOD_PROXY)
    }

    public synchronized static Hood getInstance() {
     if (mInstance == null) {
            mInstance = new  Hood();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {

         SmartDashboard.putNumber("MotorSetpoint", mPeriodicIO.demand);

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
                mState =  State.IDLE;
                // startLogging();
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (Hood.this) {
                 // What happens while the robot is on. This is usually a state machine
                }
            }

            @Override
            public void onStop(final double timestamp) {
                mRunningManual = false;
                mState = /*Whatever state you want the robot to be when it stops*/;
                stopLogging();
            }
        });
    }


    public void runStateMachine() {
        switch (mState) {
        // Cases for each state and what the actuators should be at those states
        }
    }


    public void setState(final WantedAction wanted_state) {
        switch (wanted_state) {
        // Cases to switch 
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

        // Use .set on each of your actuators to whatever output you have been setting from periodicIO. This is also a good place to add limits to your code. 
    
    }

    @Override
    public boolean checkSystem() {
    // Use CheckMotor function on any motor you have that uses PID, else just return true in this block
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser//*SUBSYSTEM XXXXXXXXX, make sure it is in CAPS*/-LOGS.csv", PeriodicIO.class);
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
        public double motorTempature;
  
        // OUTPUTS
        public double demand;
        public double isReady;
    }




}