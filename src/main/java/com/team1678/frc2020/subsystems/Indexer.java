package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.subsystems.Canifier;
import com.team1678.frc2020.subsystems.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
    private static Indexer mInstance = null;
    private Canifier mCanifier = Canifier.getInstance();
    private Turret mTurret = Turret.getInstance();

    private static final double kFeedingVoltage = 12.;
    private static final double kOuttakeVoltage = -4.;
    private static final double kIdleVoltage = 0.;
    private static final double kIndexingVelocity = 120.; // degrees per second
    private static final double kGearRatio = 200.; // TODO(Hanson) verify with design

    public static class PeriodicIO {
        // INPUTS
        public double current_encoder_ticks;
        public boolean limit_switch;
        public double current_turret_angle;

        // OUTPUTS
        public ControlMode indexer_control_mode;
        public double indexer_demand;
        public double feeder_demand;
    }

    public enum WantedAction {
        NONE, INDEX, PREP_FEED, MOVE, FEED,
    }

    public enum State {
        IDLE, INDEXING, PREPPING, MOVING, FEEDING,
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private final TalonFX mIndexer;
    private final TalonFX mFeeder;
    private State mState = State.IDLE;
    private boolean mRunningManual = false;
    private double mCurrentTheta = mPeriodicIO.current_encoder_ticks / 2048 / kGearRatio * 360;
    private double mCurrentThetaAdjusted = mCurrentTheta % 360;
    private double mCurrentTurretAngle = mPeriodicIO.current_turret_angle;
    private double mTurretAngleFromSixty = (mCurrentTurretAngle % 60 < 30) ? 
                                            mCurrentTurretAngle % 60 : mCurrentTurretAngle % 60 - 60;
    private double mThetaToFeeder = mCurrentTheta % 60 - mTurretAngleFromSixty;
    private double mDeadband = 0.5; // tune
    private double mThetaGoal = 0;
    private double mInitialTime = 0;
    private double mInitialTheta = 0;
    private double mInitialTurretAngle = 0;
    private boolean mStartCounting = false;
    private boolean mStartRotating = false;
    private double mWaitTime = .1; // seconds
    private boolean mHasBeenZeroed = false;

    private Indexer() {
        mIndexer = new TalonFX(Constants.kIndexerId);
        mFeeder = new TalonFX(Constants.kFeederId);

        mIndexer.set(ControlMode.Velocity, 0);
        mIndexer.setInverted(false);
        mIndexer.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mIndexer.enableVoltageCompensation(true);

        mFeeder.set(ControlMode.PercentOutput, 0);
        mFeeder.setInverted(false);
        mFeeder.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mFeeder.enableVoltageCompensation(true);
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
        SmartDashboard.putNumber("FeederSetpoint", mPeriodicIO.feeder_demand);
    }

    public synchronized void setOpenLoop(double percentage) {
        mRunningManual = true;
        mPeriodicIO.feeder_demand = percentage;
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
                synchronized (Indexer.this) {
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
            }
        });
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicIO.limit_switch) {
            zeroSensors();
        }
    }

    public synchronized double getIndexerTheta() {
        return mCurrentTheta;
    }

    public synchronized double getIndexerThetaAdjusted() {
        return mCurrentThetaAdjusted;
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            if (modifyOutputs) {
                mPeriodicIO.indexer_control_mode = ControlMode.PercentOutput;
                mPeriodicIO.indexer_demand = kIdleVoltage;
                mPeriodicIO.feeder_demand = kIdleVoltage;
            }
            break;
        case INDEXING:
            if (modifyOutputs) {
                mPeriodicIO.indexer_control_mode = ControlMode.Velocity;
                mPeriodicIO.indexer_demand = kIndexingVelocity;
                mPeriodicIO.feeder_demand = kOuttakeVoltage;
            }
            break;
        case PREPPING:
            if (modifyOutputs) {
                mPeriodicIO.indexer_control_mode = ControlMode.Position;
                mPeriodicIO.feeder_demand = kFeedingVoltage;

                if (mThetaToFeeder < 30 && mThetaToFeeder > -30) {
                    mThetaGoal = mThetaToFeeder;
                } else {
                    mThetaGoal = mThetaToFeeder - 60;
                }
                mPeriodicIO.indexer_demand = mCurrentTheta + mThetaGoal;

                if (mThetaToFeeder < mDeadband || 60 - mThetaToFeeder < mDeadband) {
                    mState = State.FEEDING;
                }
            }
            break;
        case MOVING:
            if (modifyOutputs) {
                mPeriodicIO.indexer_control_mode = ControlMode.Position;
                mPeriodicIO.feeder_demand = kFeedingVoltage;

                if (!mStartRotating) {
                    mInitialTheta = mCurrentTheta;
                    mInitialTurretAngle = mCurrentTurretAngle;
                    mStartRotating = true;
                    mThetaGoal = 60;
                }
                if (mStartRotating && mCurrentTheta - mInitialTheta > 60
                    + (mCurrentTurretAngle - mInitialTurretAngle) - mDeadband) {
                    mStartRotating = false;
                    mState = State.FEEDING;
                }

                mPeriodicIO.indexer_demand = mCurrentTheta + mThetaGoal;
            }
        case FEEDING:
            if (modifyOutputs) {
                mPeriodicIO.indexer_control_mode = ControlMode.Position;
                mPeriodicIO.feeder_demand = kFeedingVoltage;
                
                if (mThetaToFeeder < mDeadband || 60 - mThetaToFeeder < mDeadband) {
                    if (mThetaToFeeder < mDeadband) {
                        mThetaGoal = mThetaToFeeder;
                    } else if (60 - mThetaToFeeder < mDeadband) {
                        mThetaGoal = mThetaToFeeder - 60;
                    }

                    final double now = Timer.getFPGATimestamp();
                    if (!mStartCounting) {
                        mInitialTime = now;
                        mStartCounting = true;
                    }
                    if (mStartCounting && now - mInitialTime > mWaitTime) {
                        mState = State.MOVING;
                        mStartCounting = false;
                    }
                } else {
                    mState = State.PREPPING;
                }

                mPeriodicIO.indexer_demand = mCurrentTheta + mThetaGoal;
            }
            break;
        default:
            System.out.println("Fell through on Indexer states!");
        }
    }

    public double getFeederVoltage() {
        return mPeriodicIO.feeder_demand;
    }

    public double getIndexerVelocity() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            return mPeriodicIO.indexer_demand;
        } else {
            return 0;
        }
    }

    public void setState(WantedAction wanted_state) {
        mRunningManual = false;
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case INDEX:
            mState = State.INDEXING;
            break;
        case PREP_FEED:
            mState = State.PREPPING;
            break;
        case MOVE:
            mState = State.MOVING;
            break;
        case FEED:
            mState = State.FEEDING;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.current_encoder_ticks = mIndexer.getSelectedSensorPosition();
        mPeriodicIO.limit_switch = mCanifier.getIndexerLimit();
        mPeriodicIO.current_turret_angle = mTurret.getAngle();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.indexer_control_mode == ControlMode.Velocity) {
            mIndexer.set(mPeriodicIO.indexer_control_mode, mPeriodicIO.indexer_demand / 10 / 360 * kGearRatio * 2048);
        } else if (mPeriodicIO.indexer_control_mode == ControlMode.Position) {
            mIndexer.set(mPeriodicIO.indexer_control_mode, mPeriodicIO.indexer_demand / 360 * kGearRatio * 2048);
        }
        mFeeder.set(ControlMode.PercentOutput, mPeriodicIO.feeder_demand / 12.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}