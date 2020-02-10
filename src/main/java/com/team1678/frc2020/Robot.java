/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2020;

import java.util.Optional;

import com.team1678.frc2020.auto.AutoModeExecutor;
import com.team1678.frc2020.auto.modes.AutoModeBase;
import com.team1678.frc2020.controlboard.ControlBoard;
import com.team1678.frc2020.loops.Looper;
import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team1678.frc2020.controlboard.ControlBoard;
import com.team1678.frc2020.controlboard.GamepadButtonControlBoard;
import com.team1678.frc2020.logger.LoggingSystem;
import com.team254.lib.wpilib.TimedRobot;
import com.team1678.frc2020.SubsystemManager;
import com.team1678.frc2020.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;

import java.util.Optional;

import com.team1678.frc2020.SubsystemManager;
import com.team1678.frc2020.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2020.subsystems.Indexer.WantedAction;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */

    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();
    private final Looper mLoggingLooper = new Looper();


    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Wrangler mWrangler = Wrangler.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private boolean climb_mode = false;
    private boolean buddy_climb = false;
    private AutoModeExecutor mAutoModeExecutor;
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private LoggingSystem mLogger = LoggingSystem.getInstance();

    public Robot() {
        CrashTracker.logRobotConstruction();
        mTrajectoryGenerator.generateTrajectories();
    }

    @Override
    public void robotPeriodic() {
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();

        SmartDashboard.putBoolean("Climb Mode", climb_mode);
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                //mCanifier, 
                mDrive, 
                mLimelight, 
                mIntake, 
                mIndexer, 
                mWrangler, 
                //mShooter,
                //mTrigger,
                mSuperstructure,
                //mHood,
                mTurret,
                mClimber,
                mInfrastructure
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mLimelight.setLed(Limelight.LedMode.ON);
            mIntake.registerLogger(mLogger);
            mLogger.registerLoops(mLoggingLooper);

            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            mInfrastructure.setIsDuringAuto(true);

            mAutoModeExecutor.start();

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
    
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(false);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLimelight.setPipeline(Constants.kPortPipeline);

            mControlBoard.reset();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();

            mDrive.setCheesyishDrive(throttle, turn, mControlBoard.getQuickTurn());

            //mSuperstructure.setWantAutoAim(mControlBoard.getTurretCardinal().rotation);
            mSuperstructure.setWantAutoAim(mControlBoard.getTurretCardinal().rotation);//setWantFieldRelativeTurret(mControlBoard.getTurretCardinal().rotation);

            if (mControlBoard.climbMode()) {
                climb_mode = true;
                System.out.println("climb mode");
            }

            if (!climb_mode){ //TODO: turret preset stuff and jog turret and rumbles
                if (mIndexer.slotsFilled()) {
                    mControlBoard.setRumble(false);
                } else {
                    mControlBoard.setRumble(false);
                }
                
                if (mControlBoard.getShoot()){
                    mSuperstructure.setWantShoot();    
                } else if (mControlBoard.getSpinUp()) {
                    mSuperstructure.setWantSpinUp();
                } else if (mControlBoard.getRunIntake()) {
                    mIntake.setState(Intake.WantedAction.INTAKE);
                    mSuperstructure.setAutoIndex(true);
                } else if (mControlBoard.getRetractIntake()) {
                    mIntake.setState(Intake.WantedAction.RETRACT);
                } else if (mControlBoard.getControlPanelRotation()) {
                    mIntake.setState(Intake.WantedAction.INTAKE);
                    mSuperstructure.setAutoIndex(false);
                    mIndexer.setBackwardsMode(false);
                } else if (mControlBoard.getControlPanelPosition()) {
                    // mRoller.setState(Roller.WantedAction.ACHIEVE_POSITION_CONTROL);
                    mIntake.setState(Intake.WantedAction.INTAKE);
                    mSuperstructure.setAutoIndex(false);
                    mIndexer.setBackwardsMode(true);
                } else {
                    mIntake.setState(Intake.WantedAction.NONE);
                } 
            } else {
                mIndexer.setState(WantedAction.PREP);
                mIntake.setState(Intake.WantedAction.NONE);
                if (mControlBoard.getArmExtend()) { // Press and hold A
                    mClimber.setState(Climber.WantedAction.PIVOT);
                } else if (mControlBoard.getStopExtend()) {
                    mClimber.setState(Climber.WantedAction.STOP);
                } else if (mControlBoard.getClimberAdjust()) { // Press and hold B
                    mClimber.setState(Climber.WantedAction.EXTEND); // hook onto the rung
                /* } else if (mControlBoard.getBuddyDeploy()) { // Press X
                    mWrangler.setState(Wrangler.WantedAction.DEPLOY); */
                } else if (mControlBoard.getWrangle()) { // Press and hold X
                    mWrangler.setState(Wrangler.WantedAction.WRANGLE);
                    buddy_climb = true;
                } else if (mControlBoard.getClimb()) { // Press and hold Y
                    if (!buddy_climb) {
                        mClimber.setState(Climber.WantedAction.CLIMB);
                    } else {
                        mClimber.setState(Climber.WantedAction.BUDDY_CLIMB);
                    }
                } else if (mControlBoard.getBrake()) { // Release Y
                    mClimber.setState(Climber.WantedAction.BRAKE);
                } else if (mControlBoard.getLeaveClimbMode()) {
                    climb_mode = false;
                    buddy_climb = false;
                } else {
                    mWrangler.setState(Wrangler.WantedAction.NONE);
                    mClimber.setState(Climber.WantedAction.NONE);
                }
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            mDrive.checkSystem();
            // mCargoIntake.checkSystem();
            // mWrist.checkSystem();
            // mElevator.checkSystem();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(true);

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.triggerOutputs();

            mDrive.setBrakeMode(false);
            mLimelight.writePeriodicOutputs();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        // mLimelight.setStream(2);

        try {
            mLimelight.setLed(Limelight.LedMode.ON);

            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
