package com.team1678.frc2020.controlboard;

import com.team1678.frc2020.Constants;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final MainDriveControlBoard mDriveControlBoard;
    private final GamepadButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    public void reset() {
    }

    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    public boolean getThrust() {
        return mDriveControlBoard.getThrust();
    }

    public double getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }

    public boolean getScorePresetLow() {
        return mButtonControlBoard.getScorePresetLow();
    }

    public boolean getScorePresetMiddle() {
        return mButtonControlBoard.getScorePresetMiddle();
    }

    // Intake

    public boolean getRunIntake() {
        return mButtonControlBoard.getRunIntake();
    }

    public boolean getRunOuttake() {
        return mButtonControlBoard.getRunOuttake();
    }

    public boolean getScorePresetHigh() {
        return mButtonControlBoard.getScorePresetHigh();
    }

    public boolean getScorePresetCargo() {
        return mButtonControlBoard.getScorePresetCargo();
    }

    public boolean getPresetStow() {
        return mButtonControlBoard.getPresetStow();
    }

    public boolean getPickupDiskWall() {
        return mButtonControlBoard.getPickupDiskWall();
    }

    public boolean getPickupBallGround() {
        return mButtonControlBoard.getPickupBallGround();
    }

    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    public boolean getToggleHangMode() {
        return mButtonControlBoard.getToggleHangMode();
    }

    public boolean getToggleHangModeLow() {
        return mButtonControlBoard.getToggleHangModeLow();
    }

    public double getElevatorThrottle() {
        return mButtonControlBoard.getElevatorThrottle();
    }

    //public TurretCardinal getTurretCardinal() {
    //    return mButtonControlBoard.getTurretCardinal();
    //}

    public boolean getAutoAim() {
        return mButtonControlBoard.getAutoAim();
    }

    public double getJoggingX() {
        return mButtonControlBoard.getJoggingX();
    }

    public double getJoggingZ() {
        return mButtonControlBoard.getJoggingZ();
    }

    public boolean getInterruptAuto() {
        return mDriveControlBoard.getInterruptAuto();
    }
}
