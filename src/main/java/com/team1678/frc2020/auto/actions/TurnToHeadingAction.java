package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Drive;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeadingAction implements Action {

    private Rotation2d mTargetHeading;
    private double mTimeOut;
    private double mStartTime;
    private Drive mDrive = Drive.getInstance();

    public TurnToHeadingAction(Rotation2d heading, double timeout) {
        mTargetHeading = heading;
        mTimeOut = timeout;
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn() || ((Timer.getFPGATimestamp() - mStartTime) > mTimeOut);
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}