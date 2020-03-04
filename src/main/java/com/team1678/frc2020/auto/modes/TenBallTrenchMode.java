package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.subsystems.*;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Arrays;

import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class TenBallTrenchMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mStealToFirstShot;
    private DriveTrajectoryAction mBarIntake;
    private DriveTrajectoryAction mBarToOutsideTrench;
    private DriveTrajectoryAction mTrenchIntake;
    private DriveTrajectoryAction mTrenchToShot;


    public TenBallTrenchMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot, false);
        mBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().barIntake, false);
        mBarToOutsideTrench = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().barToOutsideTrench, false);
        mTrenchIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().trenchIntake, false);
        mTrenchToShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().trenchToShot, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 10 ball auto");

        // Prep for intake
        runAction(new ParallelAction(Arrays.asList(
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)),       
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(160.))))));

        // Drive to intake balls
        runAction(mStartToSteal);

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));

        runAction(mStealToFirstShot);
            
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0), 1.5));
        System.out.println("Turn Complete");

        runAction(new WaitUntilOnTargetAction());
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());

        runAction(new WaitForIndexerSpinAction(720.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));

        runAction(mBarIntake);

        runAction(mBarToOutsideTrench);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));
        runAction(mTrenchIntake);

        runAction(mTrenchToShot);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

        runAction(new WaitUntilOnTargetAction());
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitForSpinupAction());

        runAction(new WaitForIndexerSpinAction(720.0));

        System.out.println("Wait Complete");
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));


        System.out.println("Auto Complete");

    }
}
