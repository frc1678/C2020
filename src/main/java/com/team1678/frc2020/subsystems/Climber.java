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

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.TimeDelayedBoolean;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Climber extends Subsystem  {
    private static Climber mInstance = null;

    private static final double kIdleVoltage = 0.0;
    private static final double kBrakeVelocity = 500.0;

    private static final int kExtendDelta = (204000 - (-27600)); // needs update
    private static final int kClimbDelta = 5000; // needs update

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum WantedAction {
        NONE, EXTEND, HUG, CLIMB, BRAKE, GODMODE, STRAFE, STOP,
    }

    public enum State {
        IDLE, EXTENDING, HUGGING, CLIMBING, BRAKING, GODMODING, STRAFING,
    }

    private State mState = State.IDLE;

    private final TalonFX mElevator;
    private final TalonFX mStrafer;
    private final Solenoid mBrakeSolenoid;
    private double mHoldingPos = 0.0;
    private double mZeroPos;
    private boolean mIsAtGoal = false;
    private TimeDelayedBoolean brake_activation = new TimeDelayedBoolean();

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 40, 40, .2);
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;


    private Climber() {
        mElevator = TalonFXFactory.createDefaultTalon(Constants.kWinchMasterId);
        mElevator.set(ControlMode.PercentOutput, 0);
        mElevator.setInverted(true);
        mElevator.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mElevator.enableVoltageCompensation(true);

        mElevator.configMotionAcceleration(40000, Constants.kLongCANTimeoutMs);
        mElevator.configMotionCruiseVelocity(20000, Constants.kLongCANTimeoutMs);
        mElevator.config_kP(0, 0.5);
        mElevator.config_kI(0, 0);
        mElevator.config_kD(0, 0);
        mElevator.config_kF(0, 0.05);

        mStrafer = TalonFXFactory.createDefaultTalon(Constants.kWinchMasterId);
        mStrafer.set(ControlMode.PercentOutput, 0);
        mStrafer.setInverted(true);
        mStrafer.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mStrafer.enableVoltageCompensation(true);

        mElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mStrafer.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mElevator.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);
        mStrafer.setSelectedSensorPosition(0, 0, Constants.kLongCANTimeoutMs);

        mElevator.setNeutralMode(NeutralMode.Coast);
        mStrafer.setNeutralMode(NeutralMode.Coast);

        mElevator.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);
        mStrafer.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);

        mBrakeSolenoid = new Solenoid(Constants.kBrakeSolenoid);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public synchronized void setBrakeMode(boolean brake) {
        mElevator.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("ClimberState", mState.name());
        SmartDashboard.putNumber("ElevatorDemand", mPeriodicIO.elevator_demand);
        SmartDashboard.putNumber("ElevatorPosition", mPeriodicIO.elevator_position);
        SmartDashboard.putNumber("ElevatorCurrent", mPeriodicIO.elevator_current);
        SmartDashboard.putNumber("ElevatorVoltage", mPeriodicIO.elevator_voltage);
        SmartDashboard.putNumber("ElevatorVelocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("StraferDemand", mPeriodicIO.strafer_demand);
        SmartDashboard.putNumber("StraferPosition", mPeriodicIO.strafer_position);
        SmartDashboard.putNumber("StraferCurrent", mPeriodicIO.strafer_current);
        SmartDashboard.putNumber("StraferVoltage", mPeriodicIO.strafer_voltage);

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
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.IDLE;
                startLogging();
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
                stopLogging();
            }
        });
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CLIMBER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public synchronized State getState() {
        return mState;
    }

    public void setBrake(boolean brake) {
        mPeriodicIO.brake_solenoid = brake;
    } 

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.elevator_demand = percentage;
    }

    public void setZeroPosition() {
        mZeroPos = mPeriodicIO.elevator_position;
    }

    public boolean isAtGoal() {
        return mIsAtGoal;
    }

    public void godmode(double delta) {
        mPeriodicIO.elevator_demand = mPeriodicIO.elevator_position;
        mPeriodicIO.elevator_demand += delta;
    }

    public void strafe(double delta) {
        mPeriodicIO.strafer_demand = mPeriodicIO.strafer_position;
        mPeriodicIO.strafer_demand += delta;
    }

    public void runStateMachine() {
        final double now = Timer.getFPGATimestamp();
        switch (mState) {
        case IDLE:
            mIsAtGoal = true;
            mPeriodicIO.elevator_demand = kIdleVoltage;
            break;
        case EXTENDING:
            mPeriodicIO.elevator_demand = mZeroPos + kExtendDelta;
            mPeriodicIO.brake_solenoid = false;
            mIsAtGoal = (mPeriodicIO.elevator_position - mZeroPos) > kExtendDelta - 2000;
            break;
        case CLIMBING:
            mPeriodicIO.elevator_demand = mZeroPos + kClimbDelta;
            mPeriodicIO.brake_solenoid = false;
            mIsAtGoal = false;

            if ((Math.abs(mPeriodicIO.elevator_position - (mZeroPos + kClimbDelta)) < 5000 && Math.abs(mPeriodicIO.velocity) < kBrakeVelocity)) {
                mHoldingPos = mPeriodicIO.elevator_position;
                mIsAtGoal = true;
                mState = State.BRAKING;
            }
            mPeriodicIO.brake_solenoid = false;
            break;
        case BRAKING:
            mPeriodicIO.elevator_demand = mHoldingPos;
            mIsAtGoal = false;
            if (!mPeriodicIO.brake_solenoid) {
                if (mPeriodicIO.velocity < kBrakeVelocity) {
                    mPeriodicIO.brake_solenoid = true;
                    mIsAtGoal = true;
                }
            }
            break;
        case GODMODING:
            mPeriodicIO.brake_solenoid = false;
            break;
        case STRAFING:
            break;
        default:
            System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        if (wanted_state == WantedAction.BRAKE && mState != State.BRAKING) {
            mHoldingPos = mPeriodicIO.elevator_position;
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
        case BRAKE:
            mState = State.BRAKING;
            break;
        case GODMODE:
            mState = State.GODMODING;
            break;
        case STRAFE:
            mState = State.STRAFING;
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
        mPeriodicIO.elevator_position = mElevator.getSelectedSensorPosition(0);
        mPeriodicIO.strafer_position = mStrafer.getSelectedSensorPosition(0);
        mPeriodicIO.velocity = mElevator.getSelectedSensorVelocity(0);
        mPeriodicIO.elevator_current = mElevator.getSupplyCurrent();
        mPeriodicIO.elevator_voltage = mElevator.getMotorOutputVoltage();
        mPeriodicIO.strafer_current = mStrafer.getSupplyCurrent();
        mPeriodicIO.strafer_voltage = mStrafer.getMotorOutputVoltage();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
        //LogSend();  
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mState == State.BRAKING || mState == State.EXTENDING || mState == State.CLIMBING || mState == State.GODMODING) {
            mElevator.set(ControlMode.MotionMagic, mPeriodicIO.elevator_demand);
        } else {
            mElevator.set(ControlMode.PercentOutput, mPeriodicIO.elevator_demand / 12.0);
        }
        mStrafer.set(ControlMode.Position, mPeriodicIO.strafer_demand);

        mBrakeSolenoid.set(mPeriodicIO.brake_solenoid);
    }

    public static class PeriodicIO {
        // INPUTS
        public double elevator_position;
        public double strafer_position;
        public double velocity;
        public boolean braked;
        public double elevator_current;
        public double elevator_voltage;
        public double strafer_current;
        public double strafer_voltage;

        // OUTPUTS
        public double elevator_demand;
        public double strafer_demand;
        public boolean brake_solenoid;
    }
}
