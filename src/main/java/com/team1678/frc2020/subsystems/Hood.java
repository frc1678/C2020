package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.subsystems.Canifier;
import com.team254.lib.drivers.MotorChecker;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.team254.lib.drivers.BaseTalonChecker;

import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import java.util.ArrayList;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;
    private boolean mHoming = true;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }

        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return Canifier.getInstance().getHoodLimit();
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean getAtGoal() {
        return Util.epsilonEquals(getAngle(), getSetpoint(), 0.5);
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
        if (mHoming && atHomingLocation()) {
            mMaster.setSelectedSensorPosition((int) unitsToTicks(12.875));
            mMaster.overrideSoftLimitsEnable(true);
            System.out.println("Homed!!!");
            mHoming = false;
        }
        super.readPeriodicInputs();
    }

    @Override
    public boolean checkSystem() {
        return BaseTalonChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<BaseTalon>>() {
            private static final long serialVersionUID = -716113039054569446L;

            {
                add(new MotorChecker.MotorConfig<>("master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunOutputPercentage = 0.5;
                mRunTimeSec = 1.0;
                mCurrentFloor = 0.1;
                mRPMFloor = 90;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 200;
                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity();
            }
        });
    }
}