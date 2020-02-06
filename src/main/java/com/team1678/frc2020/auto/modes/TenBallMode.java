package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.subsystems.*;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Arrays;

import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class TenBallMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mStartToSteal;
    private DriveTrajectoryAction mStealToFirstShot;
    private DriveTrajectoryAction mIntakeCells;
    private DriveTrajectoryAction mIntakeToSecondShot;


    public TenBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot, false);
        mIntakeCells = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeCells, false);
        mIntakeToSecondShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeToSecondShot, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 10 ball auto");

        // Prep for intake
        runAction(new ParallelAction(Arrays.asList(
            new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)),       
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.))))));

        // Drive to intake balls
        runAction(mStartToSteal);

        runAction(new ParallelAction(Arrays.asList(
            mStealToFirstShot,
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true))           
        )));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
        System.out.println("Turn Complete");

        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));
        runAction(new WaitAction(1.0));
        runAction( new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitAction(2.0));
    
        
        System.out.println("Wait Complete");
        //runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));

        runAction(mIntakeCells);
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));

        runAction(new ParallelAction(Arrays.asList(
            mIntakeToSecondShot,
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true))

        )));

        runAction(mIntakeToSecondShot);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
        runAction(new WaitAction(2.0));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
        System.out.println("Auto Complete");

    }
}
