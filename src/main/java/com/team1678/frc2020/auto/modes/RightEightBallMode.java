package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.subsystems.*;
import com.team1678.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

import java.util.Arrays;

import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class RightEightBallMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mRightSideStartToBarIntake;
    private DriveTrajectoryAction mBarIntakeToShot;
    private DriveTrajectoryAction mShotToTrenchEnd;
    private DriveTrajectoryAction mTrenchToShot;

    public RightEightBallMode() {
        mRightSideStartToBarIntake = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().rightSideStartToBarIntake, true);
        mBarIntakeToShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().barIntakeToShot, false);
        mShotToTrenchEnd = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().shotToTrenchEnd, false);
        mTrenchToShot = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().trenchToShot, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running right side 8 ball auto");

          // Prep for intake
          runAction(new ParallelAction(Arrays.asList(
            new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)),       
            new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)),
            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(-150.))))));

        runAction(mRightSideStartToBarIntake);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(-150.))));
        
        runAction(mBarIntakeToShot);
         runAction(new LambdaAction(() -> Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.))));
         runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

         runAction(new WaitUntilOnTargetAction());
         runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
         runAction(new WaitForSpinupAction());
         runAction(new WaitForIndexerSpinAction(720.0));
 
         System.out.println("Wait Complete");
         runAction(new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)));
         runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));
 
        runAction(mShotToTrenchEnd);
        runAction(new LambdaAction(() -> Superstructure.getInstance().setWantPreShot(true)));
        runAction(mTrenchToShot);

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
