package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.subsystems.Turret;
import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2020.planners.IndexerMotionPlanner;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    private static Indexer mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;
    private Turret mTurret = Turret.getInstance();

    // private static final double kIndexingVelocity = 120.; // degrees per second
    private static final double kZoomingVelocity = 360.;
    private static final double kGearRatio = 200.; // TODO(Hanson) verify with design

    public static class PeriodicIO {
        // INPUTS
        public boolean front_proxy;
        public boolean right_proxy;
        public boolean left_proxy;
        public boolean back_right_proxy;
        public boolean back_left_proxy;
        public boolean limit_switch;

        public double indexer_angle;
        public double turret_angle;

        // OUTPUTS
        public ControlMode indexer_control_mode;
        public double indexer_demand;
    }

    public static class ProxyStatus {
        public boolean front_proxy;
        public boolean right_proxy;
        public boolean left_proxy;
        public boolean back_right_proxy;
        public boolean back_left_proxy;
    }

    public static class SlotStatus {
        public boolean slot_zero;
        public boolean slot_one;
        public boolean slot_two;
        public boolean slot_three;
        public boolean slot_four;

        public boolean slotsFilled() {
            return slot_zero && slot_one && slot_two && slot_three && slot_four;
        }
    }

    public enum WantedAction {
        NONE, INDEX, PREP, REVOLVE, ZOOM,
    }

    public enum State {
        IDLE, INDEXING, PREPPING, REVOLVING, ZOOMING, FEEDING,
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ProxyStatus mProxyStatus = new ProxyStatus();
    private SlotStatus mSlotStatus = new SlotStatus();
    private final TalonFX mIndexer;
    private State mState = State.IDLE;
    private double mInitialTime = 0;
    private boolean mStartCounting = false;
    private double mWaitTime = .1; // seconds
    private boolean mHasBeenZeroed = false;
    private boolean mBackwards = false;
    private int mSlotGoal;
    private boolean mIsAtDeadSpot = false;
    private DigitalInput mFrontProxy = new DigitalInput(Constants.kFrontIndexerProxy);
    private DigitalInput mRightProxy = new DigitalInput(Constants.kRightIndexerProxy);
    private DigitalInput mBackRightProxy = new DigitalInput(Constants.kBackRightIndexerProxy);
    private DigitalInput mBackLeftProxy = new DigitalInput(Constants.kBackLeftIndexerProxy);
    private DigitalInput mLeftProxy = new DigitalInput(Constants.kLeftIndexerProxy);
    private DigitalInput mLimitSwitch = new DigitalInput(Constants.kIndexerLimitSwitch);

    private Indexer() {
        mIndexer = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mIndexer.set(ControlMode.Velocity, 0);
        mIndexer.setInverted(false);
        mIndexer.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mIndexer.enableVoltageCompensation(true);

        mMotionPlanner = new IndexerMotionPlanner();
    }

    public synchronized static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("IndexerControlMode", mPeriodicIO.indexer_control_mode.name());
        SmartDashboard.putNumber("IndexerSetpoint", mPeriodicIO.indexer_demand);
        SmartDashboard.putNumber("IndexerAngle", mPeriodicIO.indexer_angle);

        SmartDashboard.putBoolean("FrontProxy", mPeriodicIO.front_proxy);
        SmartDashboard.putBoolean("RightProxy", mPeriodicIO.right_proxy);
        SmartDashboard.putBoolean("LeftProxy", mPeriodicIO.left_proxy);
        SmartDashboard.putBoolean("BackRightProxy", mPeriodicIO.back_right_proxy);
        SmartDashboard.putBoolean("BackLeftProxy", mPeriodicIO.back_left_proxy);

        SmartDashboard.putBoolean("SlotZero", mSlotStatus.slot_zero);
        SmartDashboard.putBoolean("SlotOne", mSlotStatus.slot_one);
        SmartDashboard.putBoolean("SlotTwo", mSlotStatus.slot_two);
        SmartDashboard.putBoolean("SlotThree", mSlotStatus.slot_three);
        SmartDashboard.putBoolean("SlotFour", mSlotStatus.slot_four);
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
        mPeriodicIO.indexer_demand = percentage;
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
        mIndexer.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    private void updateSlots(double indexer_angle) {
        mProxyStatus.front_proxy = mPeriodicIO.front_proxy;
        mProxyStatus.right_proxy = mPeriodicIO.right_proxy;
        mProxyStatus.left_proxy = mPeriodicIO.left_proxy;
        mProxyStatus.back_right_proxy = mPeriodicIO.back_right_proxy;
        mProxyStatus.back_left_proxy = mPeriodicIO.back_left_proxy;

        mSlotStatus = mMotionPlanner.updateSlotStatus(indexer_angle, mProxyStatus);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                mSlotGoal = mMotionPlanner.findNextSlot(mPeriodicIO.indexer_angle, mTurret.getAngle());
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Indexer.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicIO.limit_switch) {
            zeroSensors();
        }
    }

    public synchronized double getIndexerTheta() {
        return mPeriodicIO.indexer_angle;
    }

    public synchronized void setBackwardsMode(boolean backwards) {
        mBackwards = backwards;
    }

    public synchronized boolean slotsFilled() {
        return mSlotStatus.slotsFilled();
    }

    public synchronized boolean isAtDeadSpot() {
        return mIsAtDeadSpot;
    }

    public void runStateMachine() {
        final double turret_angle = mTurret.getAngle();
        final double indexer_angle = mPeriodicIO.indexer_angle;

        if (mMotionPlanner.isSnapped(indexer_angle)) {
            updateSlots(indexer_angle);
        }

        switch (mState) {
        case IDLE:
            break;
        case INDEXING:
            mPeriodicIO.indexer_control_mode = ControlMode.Position;

            if (!mSlotStatus.slotsFilled()) {
                mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle, mProxyStatus);
                mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoalToIntake(mSlotGoal, indexer_angle);

                if (mMotionPlanner.isAtGoal(mSlotGoal, indexer_angle, 0)) {
                    updateSlots(indexer_angle);

                    if (mProxyStatus.front_proxy) {
                        mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle, mProxyStatus);
                        mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoalToIntake(mSlotGoal, indexer_angle);
                    }   
                }                 
            } else {
                mSlotGoal = mMotionPlanner.findNearestSlot(indexer_angle, turret_angle);
                mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, turret_angle);
            }            
            break;
        case PREPPING:
            mPeriodicIO.indexer_control_mode = ControlMode.Position;

            mSlotGoal = mMotionPlanner.findNearestSlot(indexer_angle, turret_angle);
            mPeriodicIO.indexer_demand = mMotionPlanner.findNearestDeadSpot(indexer_angle, turret_angle);

            mIsAtDeadSpot = mMotionPlanner.isAtDeadSpot(indexer_angle, turret_angle);
            break;
        case REVOLVING:
            mPeriodicIO.indexer_control_mode = ControlMode.Position;

            if (!mBackwards) {
                mSlotGoal = mMotionPlanner.findNextSlot(indexer_angle, turret_angle);
            } else {
                mSlotGoal = mMotionPlanner.findPreviousSlot(indexer_angle, turret_angle);
            }

            mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, turret_angle);

            if (mMotionPlanner.isAtGoal(mSlotGoal, indexer_angle, turret_angle)) {
                mState = State.FEEDING;
            }
            break;
        case ZOOMING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = mBackwards ? -kZoomingVelocity : kZoomingVelocity;
            break;
        case FEEDING:
            mPeriodicIO.indexer_control_mode = ControlMode.Position;
            if (mMotionPlanner.isAtGoal(mSlotGoal, indexer_angle, turret_angle)) {
                final double now = Timer.getFPGATimestamp();
                if (!mStartCounting) {
                    mInitialTime = now;
                    mStartCounting = true;
                }
                if (mStartCounting && now - mInitialTime > mWaitTime) {
                    mState = State.REVOLVING;
                    mStartCounting = false;
                }
            }

            mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, turret_angle);
            break;
        default:
            System.out.println("Fell through on Indexer states!");
        }
    }

    public double getIndexerVelocity() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            return mPeriodicIO.indexer_demand;
        } else {
            return 0;
        }
    }

    public void setState(WantedAction wanted_state) {
        final State prev_state = mState;
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case INDEX:
            mState = State.INDEXING;
            break;
        case PREP:
            mState = State.PREPPING;
            break;
        case REVOLVE:
            mState = State.REVOLVING;
            break;
        case ZOOM:
            mState = State.ZOOMING;
            break;
        }

        if (mState != prev_state && mState != State.REVOLVING && mState != State.INDEXING) {
            mSlotGoal = mMotionPlanner.findNearestSlot(mPeriodicIO.indexer_angle, mTurret.getAngle());
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.front_proxy = mFrontProxy.get();
        mPeriodicIO.right_proxy = mRightProxy.get();
        mPeriodicIO.left_proxy = mLeftProxy.get();
        mPeriodicIO.back_right_proxy = mBackRightProxy.get();
        mPeriodicIO.back_left_proxy = mBackLeftProxy.get();
        mPeriodicIO.limit_switch = !mLimitSwitch.get();

        mPeriodicIO.indexer_angle = mIndexer.getSelectedSensorPosition() / 2048 / kGearRatio * 360;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            mIndexer.set(mPeriodicIO.indexer_control_mode, mPeriodicIO.indexer_demand / 10 / 360 * kGearRatio * 2048);
        } else if (mPeriodicIO.indexer_control_mode == ControlMode.Position) {
            mIndexer.set(mPeriodicIO.indexer_control_mode, mPeriodicIO.indexer_demand / 360 * kGearRatio * 2048);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}