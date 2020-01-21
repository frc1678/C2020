package com.team1678.frc2020.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.LatchedBoolean;

import edu.wpi.first.wpilibj.DigitalInput;

import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.BaseTalonChecker;
import com.team1678.lib.util.HallCalibration;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mHoming = true;
    public static final boolean kUseManualHomingRoutine = false;
    private HallCalibration calibration = new HallCalibration(0);
    private double mOffset = 0;
    private DigitalInput mLimitSwitch = new DigitalInput(0);
    
    private static Canifier mCanifier = Canifier.getInstance();

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }
        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean atHomingLocation() {
        final double enc = mMaster.getSelectedSensorPosition(0);
        System.out.println(!mLimitSwitch.get());
        calibration.update(enc, !mLimitSwitch.get());
        if (calibration.isCalibrated()) {
            mOffset = enc - calibration.getOffset();
            return true;
        }
        return false;
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
            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward, 0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();

        if (atHomingLocation()) {
            mMaster.setSelectedSensorPosition((int) Math.floor(mOffset));
            mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHoming = false;
        } else {
            System.out.println("Not homed");
        }
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = 1636612675181038895L; // TODO find the right number

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            { // TODO change to legit config
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
