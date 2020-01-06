package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team1678.frc2020.Constants;

public class Canifier extends Subsystem {
    private static Canifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
    }

    public synchronized static Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    public int getWristTicks() {
        return mCanifier.getQuadraturePosition();
    }

    public synchronized boolean getElevatorLimit() {
        return mPeriodicInputs.elevator_limit_;
    }

    public synchronized boolean getTurretLimit() {
        return mPeriodicInputs.turret_limit_;
    }

    public synchronized boolean getHoodLimit() {
        return mPeriodicInputs.hood_limit_;
    }

    public synchronized void resetWristEncoder() {
        mCanifier.setQuadraturePosition(0, 0 );
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);
        mPeriodicInputs.elevator_limit_ = !pins.SDA; //todo(HANSON)
        mPeriodicInputs.turret_limit_ = !pins.LIMR;
        mPeriodicInputs.hood_limit_ = !pins.LIMF;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        public boolean elevator_limit_;
        public boolean turret_limit_;
        public boolean hood_limit_;
    }
}
