package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
// import com.team1678.frc2020.constants.ClimberConstants;
// import com.team1678.frc2020.subsystems.DiskBrake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem  {
    static Climber mInstance = null;
    // private DiskBrake mDiskBrake = DiskBrake.getInstance();

    private static final double kWinchVoltage = 12.;
    private static final double kHoldingVoltage = 0.;
    private static final double kGearRatio = 200;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    // private DiskBrake.WantedAction mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;

    public enum WantedAction {
        NONE, EXTEND, WINCH, // BRAKE,
    }

    private enum State {
        IDLE, EXTENDING, WINCHING, // BRAKING,
    }

    private State mState = State.IDLE;

    private final TalonFX mMaster;
    private final TalonFX mSlave;
    private final Solenoid mArmSolenoid;

    private Climber() {
        mArmSolenoid = Constants.makeSolenoidForId(Constants.kArmSolenoidId);

        mMaster = new TalonFX(Constants.kWinchMasterId);
        mSlave = new TalonFX(Constants.kWinchSlaveId);

        mMaster.set(ControlMode.PercentOutput, 0);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        mSlave.follow(mMaster);
        mSlave.setInverted(false);
    }

    private double mCurrentHeight = mPeriodicIO.current_height / 2048 * kGearRatio;
    private double mMinHeight = -60; // inches, might need tuning

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
        SmartDashboard.putNumber("WinchVoltage", mPeriodicIO.demand);
        SmartDashboard.putNumber("CurrentHeight", mCurrentHeight);
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
                    // mDiskBrake.setState(mDiskBrakeGoal);
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

    public void forceArmRetract() {
        mPeriodicIO.arm_solenoid = false;
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case IDLE:
            // mDiskBrakeGoal = DiskBrake.WantedAction.NONE;
            mPeriodicIO.demand = kHoldingVoltage;
            break;
        case EXTENDING:
            // mDiskBrakeGoal = DiskBrake.WantedAction.NONE;
            mPeriodicIO.demand = kHoldingVoltage;
            mPeriodicIO.arm_solenoid = true;
        case WINCHING:
            // mDiskBrakeGoal = DiskBrake.WantedAction.RELEASE;
            if (mCurrentHeight > mMinHeight) {
                mPeriodicIO.demand = kWinchVoltage;
            } else {
                mPeriodicIO.demand = kHoldingVoltage;
            }
            mPeriodicIO.arm_solenoid = true;
            break;
        /* case BRAKING:
            mDiskBrakeGoal = DiskBrake.WantedAction.BRAKE;
            mPeriodicIO.demand = kHoldingVoltage;
            mPeriodicIO.arm_solenoid = true;
            break; */
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
        /* case BRAKE:
            mState = State.BRAKING;
            break; */
        default:
            System.out.println("No climber goal!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.current_height = mMaster.getSelectedSensorPosition();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        mArmSolenoid.set(mPeriodicIO.arm_solenoid);
    }

    public static class PeriodicIO {
        // INPUTS
        public double current_height;

        // OUTPUTS
        public double demand;
        public boolean arm_solenoid;
    }
}
