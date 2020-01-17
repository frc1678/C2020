package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem  {
    static Climber mInstance = null;

    private static final double kWinchVoltage = 12.;
    private static final double kHoldingVoltage = 0.;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum WantedAction {
        NONE, EXTEND, WINCH, BRAKE,
    }

    private enum State {
        IDLE, EXTENDING, WINCHING, BRAKING,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final Solenoid mArmSolenoid;
    private Solenoid mBrakeSolenoid;

    private Climber() {
        mArmSolenoid = Constants.makeSolenoidForId(Constants.kArmSolenoidId);
        mBrakeSolenoid = Constants.makeSolenoidForId(Constants.kBrakeSolenoidId);

        mMaster = new TalonFX(Constants.kWinchMasterId);
        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
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
        SmartDashboard.putNumber("WinchVoltage", mPeriodicIO.demand);
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
                    runStateMachine(true);
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

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            mPeriodicIO.demand = kHoldingVoltage;
            break;
        case EXTENDING:
            mPeriodicIO.demand = kHoldingVoltage;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
        case WINCHING:
            mPeriodicIO.demand = kWinchVoltage;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = false;
            break;
        case BRAKING:
            mPeriodicIO.demand = kHoldingVoltage;
            mPeriodicIO.arm_solenoid = true;
            mPeriodicIO.brake_solenoid = true;
            break;
        default:
            System.out.println("Fell through on Climber states!");
        }
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            break;
        case EXTEND:
            mState = State.EXTENDING;
            break;
        case WINCH:
            mState = State.WINCHING;
            break;
        case BRAKE:
            mState = State.BRAKING;
            break;
        default:
            System.out.println("No climber goal!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {   
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mArmSolenoid.set(mPeriodicIO.arm_solenoid);
        mBrakeSolenoid.set(mPeriodicIO.brake_solenoid);
    }

    public static class PeriodicIO {
        // INPUTS

        // OUTPUTS
        public double demand;
        public boolean arm_solenoid;
        public boolean brake_solenoid;
    }
}
