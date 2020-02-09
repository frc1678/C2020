package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;
import com.team1678.frc2020.subsystems.*;


public class TestPath extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mTestPath, mTestPathReversed;

    public TestPath() {
        mTestPath = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPath, true);
        mTestPathReversed = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPathReversed, false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90)));
        //runAction(mTestPath);
/*
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.INTAKE)));
        runAction(new WaitForIndexerSpinAction(720.0));
        runAction(new LambdaAction(() -> Intake.getInstance().setState(Intake.WantedAction.NONE)));

/*
    runAction(new LambdaAction(() -> Superstructure.getInstance().setWantSpinUp(true)));
    runAction(new WaitAction(3.0));
    //runAction(new WaitForSpinupAction(true));
    runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(true)));
   // runAction(new WaitAction(3));
    //runAction(new WaitForSpinupAction(true));
    runAction(new WaitAction(3.0));
    runAction(new LambdaAction(() -> Superstructure.getInstance().setWantShoot(false)));
*/

        //runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90.0)));
       // runAction(new DriveOpenLoopAction(0, 0, 0.1));
    }
}
