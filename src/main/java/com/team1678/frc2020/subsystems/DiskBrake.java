package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DiskBrake extends Subsystem {
    private static DiskBrake mInstance;

    public enum WantedAction {
        NONE, BRAKE, RELEASE,
    }

    private enum State {
        ENGAGED, DISENGAGED
    }

    private State mState = State.DISENGAGED;

    private PeriodicOutputs mPeriodicOutputs = new PeriodicOutputs();

    private final Solenoid mDiskBrakeSolenoid;

    private DiskBrake() {
        mDiskBrakeSolenoid = Constants.makeSolenoidForId(Constants.kDiskBrakeSolenoidId);
    }

    public synchronized static DiskBrake getInstance() {
        if (mInstance == null) {
            mInstance = new DiskBrake();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("DiskBrake", mPeriodicOutputs.disk_brake_solenoid_);
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.DISENGAGED;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (DiskBrake.this) {
                    runStateMachine(true);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.DISENGAGED;
            }
        });
    }

    public synchronized boolean getDiskBrakeEngaged() {
        return mDiskBrakeSolenoid.get() && mPeriodicOutputs.disk_brake_solenoid_;
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case ENGAGED:
            if (modifyOutputs) {    
                mPeriodicOutputs.disk_brake_solenoid_ = true;
            }
            break;
        case DISENGAGED:
            if (modifyOutputs) {
                mPeriodicOutputs.disk_brake_solenoid_ = false;
            }
            break;
        default:
            System.out.println("Fell through on Disk Brake states!");
        }
    }

    public void forceBrake() {
        mPeriodicOutputs.disk_brake_solenoid_ = true;
    }

    public void forceRelease() {
        mPeriodicOutputs.disk_brake_solenoid_ = false;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            break;
        case BRAKE:
            mState = State.ENGAGED;
            break;
        case RELEASE:
            mState = State.DISENGAGED;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mDiskBrakeSolenoid.set(mPeriodicOutputs.disk_brake_solenoid_);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicOutputs {
        // OUTPUTS
        public boolean disk_brake_solenoid_;
    }
}

