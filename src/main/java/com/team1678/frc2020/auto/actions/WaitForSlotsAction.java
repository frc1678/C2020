package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Waits until number of slots are empty or full before continuing action
 * 
 * @see Action
 */
public class WaitForSlotsAction implements Action {

    private Indexer mIndexer = Indexer.getInstance();
    private boolean mOpenSlot;

    private double mMaxSettleTime; // maximum time to wait
    private double mStartTime;

    public WaitForSlotsAction(boolean bool, double max_wait) {
        mOpenSlot = bool;
        mMaxSettleTime = max_wait;
    }

    @Override
    public boolean isFinished() {
        return (mOpenSlot ? mIndexer.slotsFilled() : mIndexer.slotsEmpty())
                || (Timer.getFPGATimestamp() - mStartTime) >= mMaxSettleTime;
    }

    @Override
    public void update() {
        // mIndexer.updateSlots(mIndexer.getIndexerTheta());
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }
}