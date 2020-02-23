package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.logger.LoggingSystem;
import com.team1678.frc2020.logger.LogStorage;

import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Wrangler extends Subsystem {
    public static double kWrangleVoltage = -12.;
    public static double kHoldingVoltage = 0.;
    
    private static Wrangler mInstance;
    private PeriodicOutputs mPeriodicOutputs = new PeriodicOutputs();
    private final TalonFX mMaster;
    private final Solenoid mDeployer;

    private boolean mBuddyClimb = false;

    LogStorage<PeriodicOutputs> mStorage = null;

    public enum WantedAction {
        NONE, DEPLOY, WRANGLE, RETRACT,
    }

    public enum State {
        IDLE, DEPLOYING, WRANGLING, RETRACTING,
    }

    private State mState = State.IDLE;

    public synchronized static Wrangler getInstance() {
        if (mInstance == null) {
            mInstance = new Wrangler();
        }
        return mInstance;
    }

    private Wrangler() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kWranglerId);
        mDeployer = Constants.makeSolenoidForId(Constants.kWranglerSolenoidId);

        mMaster.set(ControlMode.PercentOutput, 0.);
        mMaster.setInverted(false);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
    }

    @Override
    public void registerLogger(LoggingSystem LS) {
        LogSetup();
        LS.register(mStorage, "wrangler.csv");
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("WranglerMotorSetpoint", mPeriodicOutputs.demand);
    }

    @Override
    public void stop() {
        mPeriodicOutputs.demand = kHoldingVoltage;
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
                synchronized (Wrangler.this) {
                    runStateMachine();
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.IDLE;
            }
        });
    }

    public synchronized boolean getWranglerOut() {
        return mDeployer.get() && mPeriodicOutputs.deployer_solenoid;
    }

    public synchronized boolean isBuddyClimbing() {
        return mBuddyClimb;
    }

    public void runStateMachine() {
        switch (mState) {
        case IDLE:
            mPeriodicOutputs.demand = kHoldingVoltage;
            break;
        case DEPLOYING:
            mPeriodicOutputs.demand = kHoldingVoltage;
            mPeriodicOutputs.deployer_solenoid = true;
            break;
        case WRANGLING:
            mPeriodicOutputs.demand = kWrangleVoltage;
            mPeriodicOutputs.deployer_solenoid = true;
            break;
        case RETRACTING:
            mPeriodicOutputs.demand = kHoldingVoltage;
            mPeriodicOutputs.deployer_solenoid = false;
            break;
        default:
            System.out.println("Fell through on Wrangler states!");
        }
    }

    public void forceRetract() {
        mPeriodicOutputs.deployer_solenoid = false;
    }

    public double getVoltage() {
        return mPeriodicOutputs.demand;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case DEPLOY:
            mState = State.DEPLOYING;
            mBuddyClimb = true;
            break;
        case WRANGLE:
            mState = State.WRANGLING;
            break;
        case RETRACT:
            mState = State.RETRACTING;
            break;
        default:
            System.out.println("No Wrangler state!");
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        LogSend();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicOutputs.demand / 12.0);
        mDeployer.set(mPeriodicOutputs.deployer_solenoid);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicOutputs {
        // OUTPUTS
        public static double demand;
        public static boolean deployer_solenoid;
    }

    public void LogSetup() {
        mStorage = new LogStorage<PeriodicOutputs>();
        mStorage.setHeadersFromClass(PeriodicOutputs.class);
    }

    public void LogSend() {
        ArrayList<Double> items = new ArrayList<Double>();
        items.add(Timer.getFPGATimestamp());
        items.add(PeriodicOutputs.demand);
        items.add(PeriodicOutputs.deployer_solenoid ? 0.0 : 1.0);
        mStorage.addData(items);
    }
}
