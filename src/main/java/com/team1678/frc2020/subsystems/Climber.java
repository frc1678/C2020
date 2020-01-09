package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.constants.ClimberConstants;
import com.team1678.frc2020.subsystems.Elevator;
import com.team1678.frc2020.subsystems.DiskBrake;
import com.team1678.frc2020.subsystems.Canifier;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem  {
    static Climber mInstance = null;
    private Elevator mElevator = Elevator.getInstance();
    private DiskBrake mDiskBrake = DiskBrake.getInstance();
    private Canifier mCanifier = Canifier.getInstance();

    private double mHeightGoal = ClimberConstants.kGroundHeight;
    private DiskBrake.WantedAction mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
    
    public enum WantedAction {
        NONE, SECURE, REACH, HOOK, ADJUST, WRANGLE, CLIMB, BRAKE,
    }

    private enum State {
        IDLE, SECURING, REACHING, HOOKING, ADJUSTING, WRANGLING, CLIMBING, BRAKING,
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
        SmartDashboard.putString("ClimberState", mState.name());
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
                mState = State.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Climber.this) {
                    runStateMachine(true);
                    if (mState == State.SECURING) {
                        mElevator.setOpenLoop(0);
                    } else if (mState != State.IDLE && mState != State.SECURING) {
                        mElevator.setSetpointPositionPID(mHeightGoal, 0);
                    }
                    mDiskBrake.setState(mDiskBrakeGoal);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized State getState() {
        return mState;
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            mDiskBrakeGoal = DiskBrake.WantedAction.NONE;
            break;
        case SECURING:
            mHeightGoal = ClimberConstants.kGroundHeight;
            if (mElevator.getPosition() <= ClimberConstants.kGroundHeight + .5) {
                mDiskBrakeGoal = DiskBrake.WantedAction.BRAKE;
            }
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
                mHeightGoal = mElevator.getPosition();
                mDiskBrakeGoal = DiskBrake.WantedAction.BRAKE;
            }
            break;
        default:
            System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            break;
        case SECURE:
            mState = State.SECURING;
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
        default:
            System.out.println("No climber goal!");
        }
    }

    public synchronized void godmodeElevator(double offset) {
        mHeightGoal += offset;
        mHeightGoal = Util.limit(mHeightGoal, ClimberConstants.kElevatorMinHeight, ClimberConstants.kElevatorMaxHeight);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
    }
}
