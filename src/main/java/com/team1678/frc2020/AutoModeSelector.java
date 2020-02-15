package com.team1678.frc2020;

import com.team1678.frc2020.auto.modes.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, TEST_PATH, TEN_BALL_AUTO, TEN_BALL_TRENCH_AUTO, LEFT_EIGHT_BALL_AUTO, RIGHT_EIGHT_BALL_AUTO, RIGHT_EIGHT_BALL_NEAR_AUTO, CHARACTERIZE_DRIVE_TURN, CHARACTERIZE_DRIVE_STRAIGHT,
    }

    private DesiredMode mCachedDesiredMode = null;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("TEST PATH", DesiredMode.TEST_PATH);
        mModeChooser.addOption("TEN BALL AUTO", DesiredMode.TEN_BALL_AUTO);
        mModeChooser.addOption("TEN BALL TRENCH AUTO", DesiredMode.TEN_BALL_TRENCH_AUTO);
        mModeChooser.addOption("LEFT EIGHT BALL AUTO", DesiredMode.LEFT_EIGHT_BALL_AUTO);
        mModeChooser.addOption("RIGHT EIGHT BALL AUTO", DesiredMode.RIGHT_EIGHT_BALL_AUTO);
        mModeChooser.addOption("RIGHT EIGHT BALL NEAR AUTO", DesiredMode.RIGHT_EIGHT_BALL_NEAR_AUTO);
        mModeChooser.addOption("Characterize Drive Turn", DesiredMode.CHARACTERIZE_DRIVE_TURN);
        mModeChooser.addOption("Characterize Drive Straight", DesiredMode.CHARACTERIZE_DRIVE_STRAIGHT);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        case CHARACTERIZE_DRIVE_TURN:
            return Optional.of(new CharacterizeDrivebaseMode(false, true));
        case CHARACTERIZE_DRIVE_STRAIGHT:
            return Optional.of(new CharacterizeDrivebaseMode(false, false));
        case TEST_PATH:
            return Optional.of(new TestPath());
        case TEN_BALL_AUTO:
            return Optional.of(new TenBallMode());
        case TEN_BALL_TRENCH_AUTO:
            return Optional.of(new TenBallTrenchMode());
        case LEFT_EIGHT_BALL_AUTO:
            return Optional.of(new LeftEightBallMode());
        case LEFT_NEAR_EIGHT_BALL_AUTO:
            return Optional.of(new NearLeftEightBallMode());
        case RIGHT_EIGHT_BALL_AUTO:
            return Optional.of(new RightEightBallMode());
        case RIGHT_EIGHT_BALL_NEAR_AUTO:
            return Optional.of(new RightEightBallNearMode());
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