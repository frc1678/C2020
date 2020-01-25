package com.team1678.frc2020.controlboard;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.controlboard.GamepadButtonControlBoard;
import com.team1678.frc2020.controlboard.GamepadButtonControlBoard.TurretCardinal;
import com.team1678.frc2020.controlboard.XboxController.Side;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    private XboxController mController;
    
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
    
    public boolean getQuickTurn(){
        return mDriveControlBoard.getQuickTurn();
    }
 
    public boolean getIntake() {
        return mButtonControlBoard.getIntake();
    }

    public boolean getOuttake() {
        return mButtonControlBoard.getOuttake();
    }

    public double getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }

    // Intake

    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    public boolean getRevolve() {
        return mButtonControlBoard.getRevolve();
    }

    public boolean getSpinUp() {
        return mButtonControlBoard.getSpinUp();
    }

    public boolean getControlPanelRotation() {
        return mButtonControlBoard.getControlPanelRotation();
    }

    public boolean getControlPanelPosition() {
        return mButtonControlBoard.getControlPanelPosition();
    }

    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    public boolean getArmDeploy() {
        return mButtonControlBoard.getArmDeploy();
    } 

    public boolean getBuddyDeploy() {
        return mButtonControlBoard.getBuddyDeploy();
    }

    public boolean getClimb() {
        return mButtonControlBoard.getClimb();
    }

    public boolean getSlowClimb() {
        return mButtonControlBoard.getSlowClimb();
    }

    public boolean getWrangle() {
        return mButtonControlBoard.getWrangle();
    }

    
    public boolean climbMode() {
        return mController.getButton(XboxController.Button.LB) && mController.getButton(XboxController.Button.RB)  && 
        mController.getTrigger(XboxController.Side.LEFT) &&  mController.getTrigger(XboxController.Side.RIGHT);
    }

    public TurretCardinal getTurretCardinal() {
        return mButtonControlBoard.getTurretCardinal();
    }

    public boolean getExitClimbMode() {
        return mButtonControlBoard.getExitClimbMode(); 
    }
}
   