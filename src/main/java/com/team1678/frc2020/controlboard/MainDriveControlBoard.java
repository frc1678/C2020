package com.team1678.frc2020.controlboard;

import com.team1678.frc2020.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }

        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    public double getThrottle() {
        return mThrottleStick.getRawAxis(1);
    }

    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    public boolean getShoot() {
        return mTurnStick.getRawButton(2);
    }

    public boolean getWantsLowGear() {
        return mThrottleStick.getRawButton(2);
    }

    public boolean getThrust() {
        return mThrottleStick.getRawButton(1);
    }
}