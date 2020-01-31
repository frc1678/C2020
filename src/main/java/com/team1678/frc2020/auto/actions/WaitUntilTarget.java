package com.team1678.frc2020.auto.actions;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitUntilTarget implements Action {

    private NetworkTable mNetworkTable;
    boolean mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

    public WaitUntilTarget() { 
    }

    @Override
    public boolean isFinished() {
        return mSeesTarget;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
    }
}
