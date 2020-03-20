package com.team1678.frc2020;

import com.team1678.frc2020.auto.modes.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH, 
        //TEN_BALL_AUTO, 
        TEN_BALL_TRENCH_AUTO, 
        LEFT_EIGHT_BALL_AUTO, 
        LEFT_NEAR_EIGHT_BALL_AUTO,
        RIGHT_EIGHT_BALL_AUTO, 
        RIGHT_NEAR_EIGHT_BALL_AUTO,
        CHARACTERIZE_DRIVE_TURN, 
        CHARACTERIZE_DRIVE_STRAIGHT,
    }

    private DesiredMode mCachedDesiredMode = null;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Path", DesiredMode.TEST_PATH);
        //mModeChooser.addOption("Ten Ball Auto", DesiredMode.TEN_BALL_AUTO);
        mModeChooser.addOption("Ten Ball Trench Auto", DesiredMode.TEN_BALL_TRENCH_AUTO);
        mModeChooser.addOption("Left Eight Ball Auto", DesiredMode.LEFT_EIGHT_BALL_AUTO);
        mModeChooser.addOption("Left Near Eight Ball Auto", DesiredMode.LEFT_NEAR_EIGHT_BALL_AUTO);
        mModeChooser.addOption("Right Eight Ball Auto", DesiredMode.RIGHT_EIGHT_BALL_AUTO);
        mModeChooser.addOption("Right Near Eight Ball Auto", DesiredMode.RIGHT_NEAR_EIGHT_BALL_AUTO);
        mModeChooser.addOption("Characterize Drive Turn", DesiredMode.CHARACTERIZE_DRIVE_TURN);
        mModeChooser.addOption("Characterize Drive Straight", DesiredMode.CHARACTERIZE_DRIVE_STRAIGHT);
        SmartDashboard.putData("Auto mode", mModeChooser);
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