package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem  {
    private static Climber mInstance = null;

    private static final double kClimbVoltage = -1.;
    private static final double kSlowClimbVoltage = 1.;
    private static final double kBrakeVelocity = 10.0;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum WantedAction {
        NONE, EXTEND, CLIMB, SLOW_CLIMB, BRAKE, STOP,
    }

    private enum State {
        IDLE, EXTENDING, CLIMBING, SLOW_CLIMBING, BRAKING,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final Solenoid mArmSolenoid;
    private Solenoid mBrakeSolenoid;
    private double mHoldingPos = 0.0;
    private TimeDelayedBoolean brake_activation = new TimeDelayedBoolean();

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

        mMaster.setNeutralMode(NeutralMode.Brake);
        mSlave.setNeutralMode(NeutralMode.Brake);
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

    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            mPeriodicIO.demand = 0;
            break;
        case EXTENDING:
            mPeriodicIO.demand = 4;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case CLIMBING:
            mPeriodicIO.demand = -4;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case SLOW_CLIMBING:
            mPeriodicIO.demand = kSlowClimbVoltage;
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
        switch (wanted_state) {
        case NONE:
            break;
        case EXTEND:
            mState = State.EXTENDING;
            break;
        case CLIMB:
            mState = State.CLIMBING;
            break;
        case SLOW_CLIMB:
            mState = State.SLOW_CLIMBING;
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
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mState == State.BRAKING) {
            //if (!mPeriodicIO.braked) {
                mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand);
            //} else {
            //    mMaster.set(ControlMode.PercentOutput, 0);
            //}
        } else {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        }
        mArmSolenoid.set(mPeriodicIO.arm_solenoid);
        mBrakeSolenoid.set(mPeriodicIO.brake_solenoid);
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
}
