package com.team1678.frc2020.controlboard;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.controlboard.XboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    // private TurretCardinal mLastCardinal;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
        reset();
    }

    public double getJogTurret() {
        double jog = mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    public boolean getScorePresetLow() {
        return mController.getButton(XboxController.Button.A);
    }

    public boolean getScorePresetMiddle() {
        return mController.getButton(XboxController.Button.B);
    }

    public boolean getScorePresetHigh() {
        return mController.getButton(XboxController.Button.Y);
    }

    public boolean getScorePresetCargo() {
        return mController.getButton(XboxController.Button.X);
    }

    public boolean getPresetStow() {
        return mController.getButton(XboxController.Button.LB);
    }

    public boolean getPickupDiskWall() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    public boolean getPickupBallGround() {
        return mController.getButton(XboxController.Button.RB);
    }

    public boolean getToggleHangMode() {
        return mController.getButton(XboxController.Button.START);
    }

    public boolean getToggleHangModeLow() {
        return mController.getButton(XboxController.Button.BACK);
    }

    public double getElevatorThrottle() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public void reset() {
        // mLastCardinal = TurretCardinal.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }
/*
    public TurretCardinal getTurretCardinal() {
        int dPad = mController.getDPad();
        TurretCardinal newCardinal = dPad == -1 ? TurretCardinal.NONE
                : TurretCardinal.findClosest(Rotation2d.fromDegrees(-dPad));
        if (newCardinal != TurretCardinal.NONE && TurretCardinal.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at
            // diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != TurretCardinal.NONE
                && (mLastCardinal == TurretCardinal.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == TurretCardinal.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return TurretCardinal.NONE;
    }
*/

    public boolean getAutoAim() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    public double getJoggingX() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public double getJoggingZ() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    // Intake

    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getRunOuttake() {
        return mController.getTrigger(Side.LEFT);
    }
}