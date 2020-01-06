package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.states.ClimberConstants;
import com.team1678.frc2020.subsystems.Elevator;
import com.team1678.frc2020.subsystems.DiskBrake;
import com.team1678.frc2020.subsystems.Canifier;

public class Climber extends Subsystem  {
    static Climber mInstance = null;
    private Elevator mElevator = Elevator.getInstance();
    private DiskBrake mDiskBrake = DiskBrake.getInstance();
    private Canifier mCanifier = Canifier.getInstance();
    private PeriodicInputs mPeriodicInputs;

    private double mHeightGoal = ClimberConstants.kGroundHeight;
    private DiskBrake.WantedAction mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
    
    public enum WantedAction {
        NONE, REACH, HOOK, ADJUST, CLIMB, BRAKE, GODMODE,
    }

    private enum State {
        IDLE, REACHING, HOOKING, ADJUSTING, CLIMBING, BRAKING, GODMODING,
    }

    private State mState = State.IDLE;

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            mDiskBrakeGoal = DiskBrake.WantedAction.NONE;
            break;
        case REACHING:
            mHeightGoal = ClimberConstants.kReachHeight;
            mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            break;
        case HOOKING:
            mHeightGoal = ClimberConstants.kHookHeight;
            mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            break;
        case ADJUSTING:
            mHeightGoal = ClimberConstants.kAdjustHeight;
            if (mElevator.getPosition() >= ClimberConstants.kAdjustHeight - 1.) {
                mHeightGoal = ClimberConstants.kGroundHeight;
            }
            mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            break;
        case CLIMBING:
            mHeightGoal = ClimberConstants.kClimbHeight;
            mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            break;
        case BRAKING:
            mHeightGoal = ClimberConstants.kClimbHeight;
            if (mElevator.getPosition() <= ClimberConstants.kClimbHeight + 1.) {
                mDiskBrakeGoal = DiskBrake.WantedAction.BRAKE;
            }
            break;
        case GODMODING:
            // godmodeElevator(godmodeGoal);
            mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            break;
        default:
            System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            break;
        case REACH:
            mState = State.REACHING;
            break;
        case HOOK:
            mState = State.HOOKING;
            break;
        case ADJUST:
            mState = State.ADJUSTING;
            break;
        case CLIMB:
            mState = State.CLIMBING;
            if (mCanifier.getElevatorLimit()) {
                mState = State.BRAKING;
            }
            break;
        case BRAKE:
            mState = State.BRAKING;
            break;
        case GODMODE:
            mState = State.GODMODING;
            break;
        }
    }

    public synchronized void godmodeElevator(double offset) {
        mHeightGoal += offset;
        mHeightGoal = Math.min(mHeightGoal, ClimberConstants.kElevatorMaxHeight);
        mHeightGoal = Math.max(mHeightGoal, ClimberConstants.kElevatorMinHeight);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // mPeriodicIO.godmode_goal_ = TODO(Hanson)
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mElevator.setSetpointPositionPID(mHeightGoal, 0);
        mDiskBrake.setState(mDiskBrakeGoal);
    }

    public static class PeriodicInputs {
        // INPUTS
        public double godmode_goal_;
    }
}