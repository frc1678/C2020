package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
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
    private DriveTrajectoryAction mIntakeAndShoot;

    private double kHoodAngle = 0.0;
    private double kShooterRPM = 0.0;


    public TenBallMode() {
        mStartToSteal = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().startToSteal, true);
        mStealToFirstShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().stealToFirstShot, true);
        mIntakeAndShoot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().intakeAndShoot, true);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running 10 ball auto");

        runAction(
            new ParallelAction(Arrays.asList(
                    mStartToSteal,
                    new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)))));


        runAction(
            new ParallelAction(Arrays.asList(
                    mStealToFirstShot,
                    new SeriesAction(Arrays.asList(
                        new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.RETRACT)),
                        new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp()),                        
                        new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(0.0))))))));

        runAction(new LambdaAction(() -> Superstructure.getInstance().setGoal(kShooterRPM, kHoodAngle, 0.0)));   
        runAction(new WaitAction(1.0));

        runAction(
            new ParallelAction(Arrays.asList(
                    mIntakeAndShoot,
                    new SeriesAction(Arrays.asList(
                        new ParallelAction(Arrays.asList(
                            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
                            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp()))),                     
                        new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(0.0))))))));

        runAction(
            new ParallelAction(Arrays.asList(
                new LambdaAction(() -> Superstructure.getInstance().setGoal(kShooterRPM, kHoodAngle, 0.0)),
                new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.RETRACT)))));
                
        runAction(new WaitAction(1.0));
                    

        runAction(mStartToSteal);
        runAction(mStealToFirstShot);
        runAction(mIntakeAndShoot);
        System.out.println("Auto Complete");

    }
}
