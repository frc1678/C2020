package com.team1678.frc2020;

import com.team1678.frc2020.auto.modes.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum StartingPosition {
        LEFT_HAB_1, RIGHT_HAB_1
    }

    enum DesiredMode {
        DO_NOTHING,
        ROCKET_HATCH,
        CHARACTERIZE_DRIVE_TURN,
        CHARACTERIZE_DRIVE_STRAIGHT,
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Rocket Hatch", DesiredMode.ROCKET_HATCH);
        mModeChooser.addOption("Characterize Drive Turn", DesiredMode.CHARACTERIZE_DRIVE_TURN);
        mModeChooser.addOption("Characterize Drive Straight", DesiredMode.CHARACTERIZE_DRIVE_STRAIGHT);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left", StartingPosition.LEFT_HAB_1);
        mStartPositionChooser.addOption("Right", StartingPosition.RIGHT_HAB_1);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if(mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        boolean startOnLeft = StartingPosition.LEFT_HAB_1 == position;
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case CHARACTERIZE_DRIVE_TURN:
                return Optional.of(new CharacterizeDrivebaseMode(false, true));
            case CHARACTERIZE_DRIVE_STRAIGHT:
                return Optional.of(new CharacterizeDrivebaseMode(false, false));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }


    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}