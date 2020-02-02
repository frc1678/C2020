package com.team1678.frc2020.controlboard;

import com.team1678.frc2020.Robot;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.controlboard.CustomXboxController.Button;
import com.team1678.frc2020.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class GamepadButtonControlBoard {
    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private TurretCardinal mLastCardinal;

    private static GamepadButtonControlBoard mInstance = null;

    // Turret
    public enum TurretCardinal {
        BACK(180),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        NONE(0),
        FRONT_LEFT(30, 45),
        FRONT_RIGHT(-30, -45),
        BACK_LEFT(150, 135),
        BACK_RIGHT(210, 235);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;

        TurretCardinal(double degrees) {
            this(degrees, degrees);
        }

        TurretCardinal(double degrees, double inputDirectionDegrees) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
        }

        public static TurretCardinal findClosest(double xAxis, double yAxis) {
            return findClosest(new Rotation2d(yAxis, -xAxis, true));
        }

        public static TurretCardinal findClosest(Rotation2d stickDirection) {
            var values = TurretCardinal.values();

            TurretCardinal closest = null;
            double closestDistance = Double.MAX_VALUE;
            for (int i = 0; i < values.length; i++) {
                var checkDirection = values[i];
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = checkDirection;
                }
            }
            return closest;
        }

        public static boolean isDiagonal(TurretCardinal cardinal) {
            return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
        }
    }

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController mController;

    private GamepadButtonControlBoard() {
        mController = new CustomXboxController(Constants.kButtonGamepadPort);
        reset();
    }

    public double getJogTurret() {
        double jog = mController.getJoystick(CustomXboxController.Side.LEFT, CustomXboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public void setRumble(boolean on) { //TODO: all 5 power cells indexed
        mController.setRumble(on);
    }

    public boolean getSpinUp() {
        return mController.getController().getAButtonReleased();
    }

    public boolean getRevolve() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getShoot() {
        return mController.getController().getBButtonReleased();
    }
    
    public boolean getIntake() {
        return mController.getTrigger(CustomXboxController.Side.RIGHT);
    } 
    
    public boolean getOuttake(){
        return mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    public boolean getControlPanelRotation() {
        return mController.getButton(CustomXboxController.Button.LB);
    }

    public boolean getControlPanelPosition() {
        return mController.getButton(CustomXboxController.Button.RB);
    }

    public boolean climbMode() {
        return mController.getButton(CustomXboxController.Button.LB) && mController.getButton(CustomXboxController.Button.RB)  && 
        mController.getTrigger(CustomXboxController.Side.LEFT) &&  mController.getTrigger(CustomXboxController.Side.RIGHT);
    }

    public boolean getArmDeploy() {
        return mController.getButton(CustomXboxController.Button.A);
    }

    public boolean getBuddyDeploy() {
        return mController.getButton(CustomXboxController.Button.B);
    }

    public boolean getClimb() {
        return mController.getButton(CustomXboxController.Button.Y);
    }

    public boolean getSlowClimb() {
        return mController.getButton(CustomXboxController.Button.RB);
    }

    public boolean getWrangle() {
        return mController.getButton(CustomXboxController.Button.X);
    }

    public boolean getLeaveClimbMode() {
        return mController.getDPad() != -1;
    }

    public void reset() {
        mLastCardinal = TurretCardinal.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

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

    public boolean getAutoAim() {
        return mController.getTrigger(CustomXboxController.Side.LEFT);
    }

    public double getJoggingX() {
        double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    public double getJoggingZ() {
        double jog = mController.getJoystick(CustomXboxController.Side.RIGHT, CustomXboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    // Intake
    public boolean getRunIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    public boolean getRetractIntake() {
        return mController.getTrigger(Side.LEFT);
    }
    

}