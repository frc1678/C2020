package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Superstructure;

public class WaitUntilOnTargetAction implements Action {

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Superstructure.getInstance().isOnTarget();
    }

    @Override
    public void done() {}
}