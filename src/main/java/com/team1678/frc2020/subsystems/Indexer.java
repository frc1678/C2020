package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
<<<<<<< HEAD
import com.team1678.frc2020.logger.LogStorage;
import com.team1678.frc2020.logger.LoggingSystem;
import com.team1678.frc2020.subsystems.Canifier;
=======
>>>>>>> 4c09bd22ed727632556f93e65e2f8e1790388736
import com.team1678.frc2020.subsystems.Turret;
import com.team254.lib.drivers.TalonFXFactory;
import com.team1678.frc2020.planners.IndexerMotionPlanner;
import com.team1678.lib.util.HallCalibration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Indexer extends Subsystem {
    private static Indexer mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;
    private Turret mTurret = Turret.getInstance();

    private static final double kZoomingVelocity = 720.;
    private static final double kPassiveIndexingVelocity = 180.;
    private static final double kGearRatio = (60. / 16.) * (160. / 18.);

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
        NONE, INDEX, PASSIVE_INDEX, PREP, REVOLVE, ZOOM,
    }

    public enum State {
        IDLE, INDEXING, PASSIVE_INDEXING, PREPPING, REVOLVING, ZOOMING, FEEDING,
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ProxyStatus mProxyStatus = new ProxyStatus();
    private SlotStatus mSlotStatus = new SlotStatus();
    private final TalonFX mMaster;
    private State mState = State.IDLE;
    private double mInitialTime = 0;
    private boolean mStartCounting = false;
    private double mWaitTime = .1; // seconds
    private boolean mHasBeenZeroed = false;
    private boolean mBackwards = false;
    private int mSlotGoal;
    private boolean mIsAtDeadSpot = false;
    private DigitalInput mSlot0Proxy = new DigitalInput(Constants.kSlot0Proxy);
    private DigitalInput mSlot1Proxy = new DigitalInput(Constants.kSlot1Proxy);
    private DigitalInput mSlot2Proxy = new DigitalInput(Constants.kSlot2Proxy);
    private DigitalInput mSlot3Proxy = new DigitalInput(Constants.kSlot3Proxy);
    private DigitalInput mSlot4Proxy = new DigitalInput(Constants.kSlot4Proxy);
    private DigitalInput mLimitSwitch = new DigitalInput(Constants.kIndexerLimitSwitch);
    private HallCalibration calibration = new HallCalibration(0);
    private double mOffset = 0;

    LogStorage<PeriodicIO> mStorage = null;

    private Indexer() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.Velocity, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMotionPlanner = new IndexerMotionPlanner();
    }

    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "indexer.csv");
    }

    public synchronized static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public boolean atHomingLocation() {
        calibration.update(mPeriodicIO.indexer_angle, mPeriodicIO.limit_switch);
        if (calibration.isCalibrated()) {
            mOffset = mPeriodicIO.indexer_angle + calibration.getOffset();
            return true;
        }
        return false;
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
        mMaster.setSelectedSensorPosition(0, 0, 10);
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
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;

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
        case PASSIVE_INDEXING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = mBackwards ? -kPassiveIndexingVelocity : kPassiveIndexingVelocity;
            break;
        case PREPPING:
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;

            mSlotGoal = mMotionPlanner.findNearestSlot(indexer_angle, turret_angle);
            mPeriodicIO.indexer_demand = mMotionPlanner.findNearestDeadSpot(indexer_angle, turret_angle);

            mIsAtDeadSpot = mMotionPlanner.isAtDeadSpot(indexer_angle, turret_angle);
            break;
        case REVOLVING:
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;

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
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;
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
        case PASSIVE_INDEX:
            mState = State.PASSIVE_INDEXING;
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
        LogSend();
        mPeriodicIO.front_proxy = mSlot0Proxy.get();
        mPeriodicIO.right_proxy = mSlot1Proxy.get();
        mPeriodicIO.back_right_proxy = mSlot2Proxy.get();
        mPeriodicIO.back_left_proxy = mSlot3Proxy.get();
        mPeriodicIO.left_proxy = mSlot4Proxy.get();
        mPeriodicIO.limit_switch = !mLimitSwitch.get();

        mPeriodicIO.indexer_angle = mMaster.getSelectedSensorPosition() / 2048 / kGearRatio * 360;
        if (atHomingLocation() && !mHasBeenZeroed) {
            mMaster.setSelectedSensorPosition((int) Math.floor(mOffset));
            mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHasBeenZeroed = true;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (!mHasBeenZeroed) {
            mMaster.set(ControlMode.PercentOutput, 0.0);
        }
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.indexer_demand / 10 / 360) * kGearRatio * 2048);
        } else if (mPeriodicIO.indexer_control_mode == ControlMode.MotionMagic) {
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.indexer_demand / 360) * kGearRatio * 2048);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public void LogSetup() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }

    public void LogSend() {
        ArrayList<Double> items = new ArrayList<Double>();
        items.add(Timer.getFPGATimestamp());
        // INPUTS
        items.add(Double.valueOf(mPeriodicIO.front_proxy? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.right_proxy? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.left_proxy? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.back_right_proxy? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.back_left_proxy? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.limit_switch? 0.0 : 1.0));
        items.add(mPeriodicIO.indexer_angle);
        items.add(mPeriodicIO.turret_angle);

        // OUTPUTS
        items.add(Double.valueOf(mPeriodicIO.indexer_control_mode.toString()));
        items.add(mPeriodicIO.indexer_demand);
    }
}
