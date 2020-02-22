package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.logger.LogStorage;
import com.team1678.frc2020.logger.LoggingSystem;
import com.team1678.frc2020.subsystems.Wrangler;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Climber extends Subsystem  {
    private static Climber mInstance = null;

    private static final double kIdleVoltage = 0.0;
    private static final double kPivotVoltage = -3.0;
    private static final double kExtendVoltage = 6.0;
    private static final double kClimbVoltage = -6.0;
    private static final double kBuddyClimbVoltage = -6.0;
    private static final double kBrakeVelocity = 500.0;
    private static final double kSoloClimbWaitTime = .5;
    private static final double kBuddyClimbWaitTime = 1.3;
    private double mInitialTime;

    private static final int kSoloExtendDelta = 280000;
    private static final int kSoloHugDelta = 237000;
    private static final int kBuddyExtendDelta = 219000;
    private static final int kBuddyHugDelta = 148000;
    private static final int kSoloClimbDelta = 5000;
    private static final int kBuddyClimbDelta = 5000;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum WantedAction {
        NONE, PIVOT, EXTEND, MANUAL_EXTEND, HUG, CLIMB, MANUAL_CLIMB, BRAKE, STOP,
    }

    private enum State {
        IDLE, PIVOTING, EXTENDING, MANUAL_EXTENDING, HUGGING, CLIMBING, MANUAL_CLIMBING, BRAKING,
    }

    LogStorage<PeriodicIO> mStorage = null;

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final Solenoid mArmSolenoid;
    private Solenoid mBrakeSolenoid;
    private double mHoldingPos = 0.0;
    private double mZeroPos;
    private TimeDelayedBoolean brake_activation = new TimeDelayedBoolean();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 10, 10, .2);

    private Climber() {
        mArmSolenoid = Constants.makeSolenoidForId(Constants.kArmSolenoidId);
        mBrakeSolenoid = Constants.makeSolenoidForId(Constants.kBrakeSolenoidId);

        mMaster = TalonFXFactory.createDefaultTalon(Constants.kWinchMasterId);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mMaster.configMotionAcceleration(20000, Constants.kLongCANTimeoutMs);
        mMaster.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mMaster.config_kP(0, 0.1);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 0);
        mMaster.config_kF(0, 0.05);

        mSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kWinchSlaveId, Constants.kWinchMasterId);
        mSlave.setInverted(false);
        mSlave.follow(mMaster);

        mMaster.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);

        mMaster.setNeutralMode(NeutralMode.Coast);
        mSlave.setNeutralMode(NeutralMode.Coast);

        mMaster.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
    }
    
    @Override
    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "climber.csv");
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putBoolean("ArmExtended", mPeriodicIO.arm_solenoid);
        SmartDashboard.putBoolean("BrakeEngaged", mPeriodicIO.brake_solenoid);
        SmartDashboard.putNumber("ClimbVoltage", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberPosition", mPeriodicIO.position);
        SmartDashboard.putNumber("ClimberVelocity", mPeriodicIO.velocity);
    }

    @Override
    public void stop() {
        setOpenLoop(0);
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
                    runStateMachine();
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

    public synchronized boolean getArmExtended() {
        return mArmSolenoid.get() && mPeriodicIO.arm_solenoid;
    }

    public void setBrake(boolean brake) {
        mPeriodicIO.brake_solenoid = brake;
    } 

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public void setZeroPosition() {
        mZeroPos = mPeriodicIO.position;
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();
        boolean buddy_climb = Wrangler.getInstance().getWranglerOut();
        switch (mState) {
        case IDLE:
            mPeriodicIO.demand = kIdleVoltage;
            mPeriodicIO.arm_solenoid = true;
            break;
        case PIVOTING:
            if (!buddy_climb) {
                if (now - mInitialTime < kSoloClimbWaitTime) {
                    mPeriodicIO.arm_solenoid = false;
                } else {
                    mPeriodicIO.arm_solenoid = true;
                }
            } else {
                if (now - mInitialTime < kBuddyClimbWaitTime) {
                    mPeriodicIO.arm_solenoid = false;
                } else {
                    mPeriodicIO.arm_solenoid = true;
                }
            }

            mPeriodicIO.demand = kPivotVoltage;
            if ((Math.abs(mPeriodicIO.velocity) < kBrakeVelocity && mPeriodicIO.arm_solenoid) || mMaster.getStatorCurrent() > 10.0) {
                mPeriodicIO.demand = kIdleVoltage;
                mZeroPos = mPeriodicIO.position;
                mState = State.EXTENDING;
            }
            mPeriodicIO.brake_solenoid = false;
            break;
        case EXTENDING:
            if (!buddy_climb) {
                mPeriodicIO.demand = mZeroPos + kSoloExtendDelta;
            } else {
                mPeriodicIO.demand = mZeroPos + kBuddyExtendDelta;
            }
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case MANUAL_EXTENDING:
            mPeriodicIO.demand = kExtendVoltage;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case HUGGING:
            if (!buddy_climb) {
                mPeriodicIO.demand = mZeroPos + kSoloHugDelta;
            } else {
                mPeriodicIO.demand = mZeroPos + kBuddyHugDelta;
            }
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case CLIMBING:
            if (!buddy_climb) {
                mPeriodicIO.demand = mZeroPos + kSoloClimbDelta;
            } else {
                mPeriodicIO.demand = mZeroPos + kBuddyClimbDelta;
            }
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;

            if ((Math.abs(mPeriodicIO.position - (mZeroPos + kSoloClimbDelta)) < 5000 && Math.abs(mPeriodicIO.velocity) < kBrakeVelocity) 
                || mMaster.getStatorCurrent() > 10.0) {
                mHoldingPos = mPeriodicIO.position;
                mState = State.BRAKING;
            }
            mPeriodicIO.brake_solenoid = false;
            break;
        case MANUAL_CLIMBING:
            mPeriodicIO.demand = kClimbVoltage;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case BRAKING:
            mPeriodicIO.demand = mHoldingPos;
            mPeriodicIO.arm_solenoid = true;
            if (!mPeriodicIO.brake_solenoid) {
                if (mPeriodicIO.velocity < kBrakeVelocity) {
                    mPeriodicIO.brake_solenoid = true;
                }
            }
            break;
        default:
            System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        if (wanted_state == WantedAction.BRAKE && mState != State.BRAKING) {
            mHoldingPos = mPeriodicIO.position;
        }

        if (wanted_state == WantedAction.PIVOT && mState != State.PIVOTING) {
            mInitialTime = Timer.getFPGATimestamp();
        }
        switch (wanted_state) {
        case NONE:
            break;
        case PIVOT:
            mState = State.PIVOTING;
            break;
        case EXTEND:
            mState = State.EXTENDING;
            break;
        case MANUAL_EXTEND:
            mState = State.MANUAL_EXTENDING;
            break;
        case HUG:
            mState = State.HUGGING;
            break;
        case CLIMB:
            mState = State.CLIMBING;
            break;
        case MANUAL_CLIMB:
            mState = State.MANUAL_CLIMBING;
            break;
        case BRAKE:
            mState = State.BRAKING;
            break;
        case STOP:
            mState = State.IDLE;
            break;
        default:
            System.out.println("No climber goal!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.position = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.braked = brake_activation.update(mBrakeSolenoid.get(), 0.5);
        
        //LogSend();  
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mState == State.BRAKING || mState == State.EXTENDING || mState == State.HUGGING || mState == State.CLIMBING) {
            mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand);
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        }
        mArmSolenoid.set(!mPeriodicIO.arm_solenoid);
        mBrakeSolenoid.set(!mPeriodicIO.brake_solenoid);
    }

    public static class PeriodicIO {
        // INPUTS
        public double position;
        public double velocity;
        public boolean braked;

        // OUTPUTS
        public double demand;
        public boolean arm_solenoid;
        public boolean brake_solenoid;
    }

    public void LogSetup() {
        mStorage = new LogStorage<PeriodicIO>();
        mStorage.setHeadersFromClass(PeriodicIO.class);
    }

    public void LogSend() {
        ArrayList<Double> items = new ArrayList<Double>();
        items.add(Timer.getFPGATimestamp());
        items.add(mPeriodicIO.demand);
        items.add(Double.valueOf(mPeriodicIO.arm_solenoid? 0.0 : 1.0));
        items.add(Double.valueOf(mPeriodicIO.brake_solenoid? 0.0 : 1.0));
        mStorage.addData(items);
    }
}
