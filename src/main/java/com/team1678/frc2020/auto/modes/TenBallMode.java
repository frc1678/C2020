package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class TenBallMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mStealToFirstShot;
    private DriveTrajectoryAction mFirstShotToIntake;
    private DriveTrajectoryAction mIntakeAndShoot;
    private DriveTrajectoryAction mIntakeToSecondShot;

    public TenBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot, true);
        mFirstShotToIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().firstShotToIntake, true);
        mIntakeAndShoot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeAndShoot, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 10 ball auto");

        runAction(mStartToSteal);
        runAction(mStealToFirstShot);
        runAction(mFirstShotToIntake);
        //runAction(mIntakeAndShoot);
        System.out.println("Auto Complete");

    }
}
