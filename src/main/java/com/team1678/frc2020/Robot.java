/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2020;

import java.util.Optional;

import com.team1678.frc2020.auto.AutoModeExecutor;
import com.team1678.frc2020.auto.modes.AutoModeBase;
import com.team1678.frc2020.controlboard.ControlBoard;
import com.team1678.frc2020.loops.Looper;
import com.team1678.frc2020.paths.TrajectoryGenerator;
// import com.team1678.frc2020.subsystems.Climber;
import com.team1678.frc2020.subsystems.Drive;
import com.team1678.frc2020.subsystems.Indexer;
import com.team1678.frc2020.subsystems.Infrastructure;
import com.team1678.frc2020.subsystems.Intake;
import com.team1678.frc2020.subsystems.LEDs;
import com.team1678.frc2020.subsystems.Limelight;
import com.team1678.frc2020.subsystems.Shooter;
import com.team1678.frc2020.subsystems.Superstructure.TurretControlModes;
// import com.team1678.frc2020.subsystems.Wrangler;
import com.team1678.frc2020.controlboard.ControlBoard;
import com.team254.lib.wpilib.TimedRobot;

import java.util.Optional;

import com.team1678.frc2020.SubsystemManager;
import com.team1678.frc2020.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2020.subsystems.RobotStateEstimator;
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

    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    // private final Climber mClimber = Climber.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    // private final Wrangler mWrangler = Wrangler.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

    private AutoModeExecutor mAutoModeExecutor;
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    public Robot() {
        CrashTracker.logRobotConstruction();
        mTrajectoryGenerator.generateTrajectories();
    }

    @Override
    public void robotPeriodic() {
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();

        boolean seesTarget = mLimelight.seesTarget();
        boolean isAimed = mSuperstructure.isAimed();
        Superstructure.TurretControlModes turretMode = mSuperstructure.getTurretControlMode();
        boolean spunUp = mShooter.spunUp();
        boolean slotsFilled = mIndexer.slotsFilled();
        Indexer.State indexerState = mIndexer.getState();
        // Climber.State climberState = mClimber.getState();
        // boolean buddyClimb = mWrangler.isBuddyClimbing();

        LEDs.State ledState = LEDs.State.OFF;

        if (isDisabled()) {
            ledState = LEDs.State.DISABLED;
        }

        if (isEnabled()) {
            ledState = LEDs.State.ENABLED;
        }

        if (slotsFilled) {
            if (!seesTarget) {
                ledState = LEDs.State.SLOTS_FILLED;
            } else {
                if (turretMode == TurretControlModes.VISION_AIMED) {
                    ledState = LEDs.State.TARGET_TRACKING;
                } else if (isAimed) {
                    ledState = LEDs.State.AT_TARGET;
                } else {
                    ledState = LEDs.State.TARGET_VISIBLE;
                }
            }
        }

        if (spunUp && seesTarget) {
            if (indexerState == Indexer.State.ZOOMING) {
                ledState = LEDs.State.SHOOTING;
            } else {
                ledState = LEDs.State.SPUN_UP;
            }
        }

        /*if (climberState == Climber.State.WINCHING) {
            if (buddyClimb) {
                ledState = LEDs.State.BUDDY_CLIMBING;
            } else {
                ledState = LEDs.State.CLIMBING;
            }
        }*/

        mLEDs.conformToState(ledState);
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(mRobotStateEstimator, mDrive, mLimelight, mIntake, mSuperstructure,
                    mTurret);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity(),0.0);
            mDrive.setHeading(Rotation2d.identity());

            mLimelight.setLed(Limelight.LedMode.OFF);

            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
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

            mDrive.setCheesyishDrive(throttle, -turn, mControlBoard.getQuickTurn());

            if (mControlBoard.getRunIntake()) {
                mIntake.setState(Intake.WantedAction.INTAKE);
            } else if (mControlBoard.getRunOuttake()) {
                mIntake.setState(Intake.WantedAction.OUTTAKE);
            } else {
                mIntake.setState(Intake.WantedAction.NONE);
            }

            mSuperstructure.setWantFieldRelativeTurret(mControlBoard.getTurretCardinal().rotation);

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

            mLimelight.setLed(Limelight.LedMode.OFF);
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
            mLimelight.setLed(Limelight.LedMode.OFF);

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
