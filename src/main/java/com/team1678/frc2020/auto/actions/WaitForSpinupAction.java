package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Superstructure;


/**
 * Waits until the shooter and feeder are up to speed, used befoer shooting
 * 
 * @see Action
 */
public class WaitForSpinupAction implements Action {
    private Superstructure mSuperstrucure = Superstructure.getInstance();
    

    public WaitForSpinupAction() {
    }

    @Override
    public boolean isFinished() {
        return mSuperstrucure.spunUp();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }
}