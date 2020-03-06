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

    private int mDPadUp = -1;
    private int mDPadDown = -1;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    public double getThrottle() {
        return -mThrottleStick.getRawAxis(1);
    }

    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(5);
    }

    public boolean getShoot() {
        return mTurnStick.getRawButton(2);
    }

    public boolean getTuck() {
        return mThrottleStick.getRawButton(2);
    }

    public boolean getManualFastRoller() {
        return mThrottleStick.getRawButton(4);
    }

    public boolean getManualSlowRoller() {
        return mThrottleStick.getRawButton(3);
    }

    public boolean getStopManualRoller() {
        return mThrottleStick.getRawButtonReleased(3) || mThrottleStick.getRawButtonReleased(4);
    }

    public boolean getShotUp() {
        int pov = mThrottleStick.getPOV();

        if (pov != mDPadUp) {
            mDPadUp = pov;
            return pov == 0;
        }
        return false;
    }
    
    public boolean getShotDown() {
        int pov = mThrottleStick.getPOV();

        if (pov != mDPadDown) {
            mDPadDown = pov;
            return pov == 180;
        }
        return false;
    }
}