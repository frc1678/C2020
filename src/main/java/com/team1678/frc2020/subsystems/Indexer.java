package com.team1678.frc2020.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.subsystems.Turret;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.Util;
import com.team1678.frc2020.planners.IndexerMotionPlanner;
import com.team1678.lib.util.HallCalibration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    private static Indexer mInstance = null;
    private IndexerMotionPlanner mMotionPlanner;
    private Turret mTurret = Turret.getInstance();

    private static final double kZoomingVelocity = 15.;
    private static final double kPassiveIndexingVelocity = 45.0;
    private static final double kGearRatio = (60. / 16.) * (160. / 16.);

    public static class PeriodicIO {
        // INPUTS
        private boolean[] raw_slots = {false, false, false, false, false};
        public boolean limit_switch;

        public double indexer_angle;
        public double indexer_velocity;
        public double turret_angle;

        // OUTPUTS
        public ControlMode indexer_control_mode = ControlMode.PercentOutput;
        public double indexer_demand;
    }

    public enum WantedAction {
        NONE, INDEX, PASSIVE_INDEX, PREP, REVOLVE, ZOOM,
    }

    public enum State {
        IDLE, INDEXING, PASSIVE_INDEXING, PREPPING, REVOLVING, ZOOMING, FEEDING,
    }

    private boolean mGeneratedGoal = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean[] mCleanSlots = {false, false, false, false, false};
    private final TalonFX mMaster;
    private State mState = State.IDLE;
    private double mInitialTime = 0;
    private boolean mStartCounting = false;
    private double mWaitTime = .1; // seconds
    private boolean mHasBeenZeroed = false;
    private boolean mBackwards = false;
    private int mSlotGoal;
    private DigitalInput mSlot0Proxy = new DigitalInput(Constants.kSlot0Proxy);
    private DigitalInput mSlot1Proxy = new DigitalInput(Constants.kSlot1Proxy);
    private DigitalInput mSlot2Proxy = new DigitalInput(Constants.kSlot2Proxy);
    private DigitalInput mSlot3Proxy = new DigitalInput(Constants.kSlot3Proxy);
    private DigitalInput mSlot4Proxy = new DigitalInput(Constants.kSlot4Proxy);
    private DigitalInput mLimitSwitch = new DigitalInput(Constants.kIndexerLimitSwitch);
    private HallCalibration calibration = new HallCalibration(0);
    private double mOffset = 0;
    private double mAngleGoal = 0;

    private Indexer() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIndexerId);

        mMaster.config_kP(0, Constants.kIndexerKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, Constants.kIndexerKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, Constants.kIndexerKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, Constants.kIndexerKf, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(1, Constants.kIndexerVelocityKp, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(1, Constants.kIndexerVelocityKi, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(1, Constants.kIndexerVelocityKd, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(1, Constants.kIndexerVelocityKf, Constants.kLongCANTimeoutMs);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(Constants.kIndexerMaxVelocity);
        mMaster.configMotionAcceleration(Constants.kIndexerMaxAcceleration);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);

        mMotionPlanner = new IndexerMotionPlanner();
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
        SmartDashboard.putBoolean("Indexer Calibrated", calibration.isCalibrated());
        SmartDashboard.putNumber("IndexerVelocity", mPeriodicIO.indexer_velocity);

        SmartDashboard.putNumber("SlotNumberGoal", mSlotGoal);

        SmartDashboard.putString("DirtySlots", Arrays.toString(mPeriodicIO.raw_slots));
        SmartDashboard.putString("CleanSlots", Arrays.toString(mCleanSlots));
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
        mCleanSlots = mMotionPlanner.updateSlotStatus(indexer_angle, mPeriodicIO.raw_slots);
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
        return false;
    }

    public synchronized boolean isAtDeadSpot() {
        return Math.abs(mPeriodicIO.indexer_angle % 72) - 36 < Constants.kIndexerDeadband;
    }

    public void runStateMachine() {
        final double turret_angle = mTurret.getAngle();
        final double indexer_angle = mPeriodicIO.indexer_angle;

        switch (mState) {
        case IDLE:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = 0;
            break;
        case INDEXING:
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;

            if (!mGeneratedGoal) {
                mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle);
                mGeneratedGoal = true;
            }
            mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, 0);

            if (mMotionPlanner.isAtGoal(mSlotGoal, indexer_angle, 0)) {
                if (mCleanSlots[mSlotGoal]) {
                    mGeneratedGoal = false;
                    // mSlotGoal = mMotionPlanner.findNearestOpenSlot(indexer_angle, mProxyStatus);
                    // mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoalToIntake(mSlotGoal,
                    // indexer_angle);
                }
            }
            break;
        case PASSIVE_INDEXING:
            mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
            mPeriodicIO.indexer_demand = mBackwards ? -kPassiveIndexingVelocity : kPassiveIndexingVelocity;
            break;
        case PREPPING:
            mPeriodicIO.indexer_control_mode = ControlMode.MotionMagic;
            mPeriodicIO.indexer_demand = mMotionPlanner.findAngleGoal(mSlotGoal, indexer_angle, turret_angle) + 36.0;
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
        mPeriodicIO.raw_slots[0] = mSlot0Proxy.get();
        mPeriodicIO.raw_slots[1] = mSlot1Proxy.get();
        mPeriodicIO.raw_slots[2] = mSlot2Proxy.get();
        mPeriodicIO.raw_slots[3] = mSlot3Proxy.get();
        mPeriodicIO.raw_slots[4] = mSlot4Proxy.get();
        mPeriodicIO.limit_switch = !mLimitSwitch.get();
        mPeriodicIO.indexer_velocity = mMaster.getSelectedSensorVelocity(0) * 600. / 2048. / kGearRatio;

        mPeriodicIO.indexer_angle = mMaster.getSelectedSensorPosition(0) / 2048. / kGearRatio * 360.;
        final double indexer_angle = mPeriodicIO.indexer_angle;

        if (mMotionPlanner.isSnapped(indexer_angle)) {
            updateSlots(indexer_angle);
        }
//        if (atHomingLocation() && !mHasBeenZeroed) {
//            mMaster.setSelectedSensorPosition((int) Math.floor(mOffset));
//            mMaster.overrideSoftLimitsEnable(true);
//            System.out.println("Homed!!!");
//            mHasBeenZeroed = true;
//        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {        
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            mMaster.selectProfileSlot(1, 0);
            System.out.println("Real demand: " + mPeriodicIO.indexer_demand);
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.indexer_demand / 600.0) * kGearRatio * 2048.0);
        } else if (mPeriodicIO.indexer_control_mode == ControlMode.MotionMagic) {
            mMaster.selectProfileSlot(0, 0);
            mMaster.set(mPeriodicIO.indexer_control_mode, (mPeriodicIO.indexer_demand / 360.0) * kGearRatio * 2048.0);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}