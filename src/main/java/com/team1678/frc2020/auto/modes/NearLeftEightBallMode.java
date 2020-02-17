package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.subsystems.*;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Arrays;

import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class NearLeftEightBallMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mLeftStealToNearFirstShot;
    private DriveTrajectoryAction mNearFirstShotToBarIntake;
    private DriveTrajectoryAction mFirstToPreSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntake;
    private DriveTrajectoryAction mSecondBarIntakeToNearShot;

    public NearLeftEightBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mLeftStealToNearFirstShot =  new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().leftStealToNearFirstShot, false);
        mNearFirstShotToBarIntake =  new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().nearFirstShotToBarIntake, false);
        mFirstToPreSecondBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().firstToPreSecondBarIntake, true);
        mSecondBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().secondBarIntake, false);
        mSecondBarIntakeToNearShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().secondBarIntakeToNearShot, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running near left side 8 ball auto");

        // Prep for intake
        runAction(new ParallelAction(Arrays.asList(
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)),       
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.))))));

        runAction(mStartToSteal);

        runAction(mLeftStealToNearFirstShot);

        // runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(700.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));

        runAction(mNearFirstShotToBarIntake);

        runAction(mFirstToPreSecondBarIntake);

        runAction(mSecondBarIntake);

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)));

        runAction(mSecondBarIntakeToNearShot);

        // runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());
        runAction(new WaitForIndexerSpinAction(700));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        
        System.out.println("Auto Complete");
    }
}
