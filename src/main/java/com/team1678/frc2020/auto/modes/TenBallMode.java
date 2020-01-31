package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class TenBallMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mStealToFirstShot;
    private DriveTrajectoryAction mFirstShotToIntake;
    private DriveTrajectoryAction mIntakeCells;
    private DriveTrajectoryAction mIntakeToSecondShot;

    public TenBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot);
        mFirstShotToIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().firstShotToIntake);
        mIntakeCells = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeCells);
        mIntakeToSecondShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeToSecondShot);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 10 ball auto");

        new WaitAction(1.0);
        runAction(mStartToSteal);
        new WaitAction(1.0);
        runAction(mStealToFirstShot);
        new WaitAction(1.0);
        runAction(mFirstShotToIntake);
        new WaitAction(1.0);
        runAction(mIntakeCells);
        new WaitAction(1.0);
        runAction(mIntakeToSecondShot);
        new WaitAction(1.0);
    }
}
