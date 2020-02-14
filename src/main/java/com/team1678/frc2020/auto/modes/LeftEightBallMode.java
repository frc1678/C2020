package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.subsystems.*;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Arrays;

import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class LeftEightBallMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mStealToOffsetFirstShot;
    private DriveTrajectoryAction mOffsetShotToFirstBarIntake;
    private DriveTrajectoryAction mFirstToPreSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntakeToShot;

    public LeftEightBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToOffsetFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToOffsetFirstShot, false);
        mOffsetShotToFirstBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().offsetShotToFirstBarIntake, false);
        mFirstToPreSecondBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().firstToPreSecondBarIntake, false);
        mSecondBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().secondBarIntake, false);
        mSecondBarIntakeToShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().secondBarIntakeToShot, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running left side 8 ball auto");

        // Prep for intake
        runAction(new ParallelAction(Arrays.asList(
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)),       
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.))))));

        runAction(mStartToSteal);

        runAction(mStealToOffsetFirstShot);

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(360.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));

        runAction(new ParallelAction(Arrays.asList(
            mOffsetShotToFirstBarIntake,
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true))
            )));
            
        runAction(mFirstToPreSecondBarIntake);

        runAction(mSecondBarIntake);

        runAction(mSecondBarIntakeToShot);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));
        
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(360.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));

        System.out.println("Auto Complete");
    }
}
