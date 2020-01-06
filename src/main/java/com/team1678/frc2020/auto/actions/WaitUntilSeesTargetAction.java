package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;

public class WaitUntilSeesTargetAction implements Action {
    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getAimingParameters(RobotState.getInstance().useHighTarget(), -1, Constants.kMaxGoalTrackAge).isPresent();
    }

    @Override
    public void done() {}
}