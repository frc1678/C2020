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
    private DriveTrajectoryAction mStealToFirstShot;
    private DriveTrajectoryAction mOffsetShotToFirstBarIntake;
    private DriveTrajectoryAction mFirstToPreSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntakeToShot;

    public LeftEightBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot =  new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot, false);
        //mStealToOffsetFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToOffsetFirstShot, false);
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
            new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(150.))))));

        runAction(mStartToSteal);

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));

        runAction(mStealToFirstShot);

        
        // runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

        runAction(new WaitUntilOnTargetAction());
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(740.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));

        runAction(mOffsetShotToFirstBarIntake);

        runAction(mFirstToPreSecondBarIntake);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));

        runAction(mSecondBarIntake);

        runAction(mSecondBarIntakeToShot);

        // runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

        runAction(new WaitUntilOnTargetAction());
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(720.0));

        System.out.println("Wait Complete");
        
        System.out.println("Auto Complete");
    }
}
