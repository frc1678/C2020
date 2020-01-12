package com.team1678.frc2020.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonFXChecker;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mHoming = true;
    public static final boolean kUseManualHomingRoutine = false;
    
    private static Canifier canifier = Canifier.getInstance();

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }
        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);
        TalonUtil.checkError(
                mMaster.configClosedLoopPeakOutput(1, 0.8, Constants.kLongCANTimeoutMs),
                "Unable to configure close loop peak output for turret!");
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean atHomingLocation() {
        return canifier.getTurretLimit();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && kUseManualHomingRoutine) {
            System.out.println("Turret going into home mode!");
            mHoming = true;
            mMaster.overrideSoftLimitsEnable(false);
        }
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
            if (atHomingLocation()) {
                mMaster.setSelectedSensorPosition((int) unitsToTicks(0));
                mMaster.overrideSoftLimitsEnable(true);
                System.out.println("Homed!!!");
                mHoming = false;
            }

            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                        0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward,
                        0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public boolean checkSystem() {
        return TalonFXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonFX>>() {
                    private static final long serialVersionUID = 1636612675181038895L;  // TODO find the right number

					{
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {   // TODO change to legit config
                        mRunOutputPercentage = 0.1;
                        mRunTimeSec = 1.0;
                        mCurrentFloor = 0.1;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = mMaster::getSelectedSensorVelocity;
                    }
                });
    }
}
