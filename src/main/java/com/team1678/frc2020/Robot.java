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
import com.team1678.frc2020.controlboard.GamepadButtonControlBoard.TurretCardinal;
import com.team1678.frc2020.logger.LoggingSystem;
import com.team254.lib.wpilib.TimedRobot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import com.team1678.frc2020.subsystems.LEDs.State;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
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
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
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

    private final Roller mRoller = Roller.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();
    private final LEDs mLEDs = LEDs.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
    private boolean climb_mode = false;
    private AutoModeExecutor mAutoModeExecutor;
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private boolean mPivoted = false;

    // private LoggingSystem mLogger = LoggingSystem.getInstance();

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
        SmartDashboard.putBoolean("Pivoted", mPivoted);
        SmartDashboard.putString("LEDs State", mLEDs.getState().name());
    }

    @Override
    public void robotInit() {
        try {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
            MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
            cameraServer.setSource(camera);

            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mCanifier,
                mDrive, 
                mHood,
                mLimelight, 
                mIntake, 
                mIndexer, 
                mWrangler, 
                mShooter,
                mTrigger,
                mSuperstructure,
                mTurret,
                mInfrastructure,
                mClimber,
                mRoller,
                mLEDs
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mLimelight.setLed(Limelight.LedMode.OFF);

           mRoller.setGameData(DriverStation.getInstance().getGameSpecificMessage());

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
            mLimelight.setLed(Limelight.LedMode.ON);

            mLimelight.setPipeline(Constants.kPortPipeline);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mInfrastructure.setIsDuringAuto(true);

            mAutoModeExecutor.start();

            mEnabledLooper.start();

            mTurret.cancelHoming();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
        //mLimelight.setLed(Limelight.LedMode.ON);

        if (!mLimelight.limelightOK()) {
            mLEDs.conformToState(LEDs.State.EMERGENCY);
        } else if (mSuperstructure.isOnTarget()) {
            mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
        } else if (mSuperstructure.getLatestAimingParameters().isPresent()) {
            mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
        } else {
            mLEDs.conformToState(LEDs.State.ENABLED);
        }

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
            mClimber.setBrakeMode(true);


            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(false);

            //mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLimelight.setLed(Limelight.LedMode.ON);
            mLimelight.setPipeline(Constants.kPortPipeline);
            mTurret.setNeutralMode(NeutralMode.Brake);
            mHood.setNeutralMode(NeutralMode.Brake);
            mLEDs.conformToState(LEDs.State.ENABLED);
            mTurret.cancelHoming();
            
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
            double hood_jog = mControlBoard.getJogHood();
            double climber_jog = mControlBoard.getJogClimber();
            double strafe = mControlBoard.getStrafe();
            Rotation2d turret_jog = mControlBoard.getJogTurret();

            if (!climb_mode) {
                if (!mLimelight.limelightOK()) {
                    mLEDs.conformToState(LEDs.State.EMERGENCY);
                } else if (mSuperstructure.getTucked()) {
                    mLEDs.conformToState(LEDs.State.HOOD_TUCKED);
                } else if (mSuperstructure.isOnTarget() && mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.TARGET_TRACKING);
                } else if (mSuperstructure.isOnTarget()) {
                    mLEDs.conformToState(LEDs.State.INVISIBLE_TARGET_TRACKING);
                } else if (mSuperstructure.getLatestAimingParameters().isPresent() && !mLimelight.seesTarget() && !mSuperstructure.getScanningHood()) {
                    mLEDs.conformToState(LEDs.State.TARGET_VISIBLE);
                } else if (mLimelight.seesTarget()) {
                    mLEDs.conformToState(LEDs.State.LIMELIGHT_SEES_ONLY);
                } else {
                    mLEDs.conformToState(LEDs.State.ENABLED);
                }
            }

            if (mControlBoard.getShotUp()) {
                mSuperstructure.setAngleAdd(1.0);
            } else if (mControlBoard.getShotDown()) {
                mSuperstructure.setAngleAdd(-1.0);
            }

            mDrive.//setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn()));
                setCheesyishDrive(throttle, turn, mControlBoard.getQuickTurn());

            //mLimelight.setLed(Limelight.LedMode.ON);
            TurretCardinal cardinal = mControlBoard.getTurretCardinal();
            
            

            // mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));//mControlBoard.getTurretCardinal().rotation);

            if (mControlBoard.climbMode()) {
                climb_mode = true;
                mPivoted = false;
            }

            if (!climb_mode){ //TODO: turret preset stuff and jog turret and rumbles
                mSuperstructure.enableIndexer(true);
                mSuperstructure.setWantUnjam(mControlBoard.getWantUnjam());
                mWrangler.setState(Wrangler.WantedAction.RETRACT);

                mSuperstructure.setManualZoom(mControlBoard.getManualZoom());

                if (mSuperstructure.getWantShoot()) {
                    mControlBoard.setRumble(true);
                } else {
                    mControlBoard.setRumble(false);
                }

                mSuperstructure.setWantHoodScan(mControlBoard.getWantHoodScan());

                if (turret_jog != null) {
                    mSuperstructure.setWantFieldRelativeTurret(
                       turret_jog.rotateBy(Rotation2d.fromDegrees(90.0)));
                } else if (mControlBoard.getFendorShot()) {
                    mSuperstructure.setWantFendor();
                    //mSuperstructure.setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.));
                } else if (cardinal == TurretCardinal.NONE) {
                    mSuperstructure.setWantAutoAim(Rotation2d.fromDegrees(180.0));
                } else {
                    mSuperstructure.setWantFieldRelativeTurret(cardinal.rotation);
                }

                if (mControlBoard.getShoot()) {
                    if (mSuperstructure.isAimed() || mSuperstructure.getWantFendor() || mSuperstructure.getWantSpit() || mSuperstructure.getLatestAimingParameters().isEmpty()) {
                        mSuperstructure.setWantShoot();
                    }
                } else if (mControlBoard.getPreShot()) {
                    mSuperstructure.setWantPreShot(true);
                } else if (mControlBoard.getSpinUp()) {
                    mSuperstructure.setWantSpinUp();
                } else if (mControlBoard.getTuck()) {
                    mSuperstructure.setWantTuck(true);
                } else if (mControlBoard.getUntuck()) {
                    mSuperstructure.setWantTuck(false);
                } else if (mControlBoard.getTurretReset()) {
                    mRobotState.resetVision();
                    mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
                } else if (mControlBoard.getTestSpit()) {
                    mSuperstructure.setWantTestSpit();
                } else if (mControlBoard.getRunIntake()) {
                    if (!mSuperstructure.getWantShoot()) {
                        mIntake.setState(Intake.WantedAction.INTAKE);
                    } else {
                        mIntake.setState(Intake.WantedAction.STAY_OUT);
                    }
                    mSuperstructure.setAutoIndex(false);
                } else if (mControlBoard.getRetractIntake()) {
                    mIntake.setState(Intake.WantedAction.RETRACT);
                //} else if (mControlBoard.getControlPanelRotation()) {
                //    mRoller.setState(Roller.WantedAction.ACHIEVE_ROTATION_CONTROL);
                //} else if (mControlBoard.getControlPanelPosition()) {
                //    mRoller.setState(Roller.WantedAction.ACHIEVE_POSITION_CONTROL);
                } else if (mControlBoard.getManualFastRoller()) {
                    mRoller.runManual(-5.0);
                } else if (mControlBoard.getManualSlowRoller()) {
                    mRoller.runManual(-2.0);
                } else {
                    mIntake.setState(Intake.WantedAction.NONE);
                    mRoller.stop();
                }
            } else {
                Climber.WantedAction climber_action = Climber.WantedAction.NONE;
                mSuperstructure.enableIndexer(false);
                mIntake.setState(Intake.WantedAction.NONE);
                mSuperstructure.setWantSpinUp(false);
                mSuperstructure.setWantShoot(false);
                mSuperstructure.setWantPreShot(false);
                mSuperstructure.setWantUnjam(false);
                mRoller.stop();

                if (mControlBoard.getArmExtend()) { // Press A
                    climber_action = (Climber.WantedAction.EXTEND);
                } else if (mControlBoard.getClimb()) { // Press Y
                    climber_action = (Climber.WantedAction.CLIMB);
                } else if (mControlBoard.getBrake()) { // Release Y
                    climber_action = (Climber.WantedAction.BRAKE);
                } else if (Math.abs(climber_jog) > Constants.kJoystickJogThreshold) { // Left joystick up/down
                    climber_jog = (climber_jog - Math.signum(climber_jog) * Constants.kJoystickJogThreshold)
                            / (1.0 - Constants.kJoystickJogThreshold);
                    climber_action = (Climber.WantedAction.GODMODE);
                    mClimber.godmode(climber_jog * 3); // probably needs tuning
                } else if (Math.abs(strafe) > Constants.kJoystickJogThreshold) { // Right joystick left/right
                    strafe = (strafe - Math.signum(strafe) * Constants.kJoystickJogThreshold)
                            / (1.0 - Constants.kJoystickJogThreshold);
                    climber_action = (Climber.WantedAction.STRAFE);
                    mClimber.strafe(strafe * 3); // probably needs tuning
                } else if (mControlBoard.getLeaveClimbMode()) {
                    climb_mode = false;
                }

                if (mClimber.getState() == Climber.State.EXTENDING || mClimber.getState() == Climber.State.CLIMBING) {
                    mLEDs.conformToState(mClimber.isAtGoal() ? LEDs.State.CLIMB_MODE : LEDs.State.CLIMB_MOTOR_RUNNING);
                } else if (mClimber.getState() == Climber.State.GODMODING && Math.abs(climber_jog) > Constants.kJoystickJogThreshold) {
                    mLEDs.conformToState(LEDs.State.CLIMB_MOTOR_RUNNING);
                } else if (mClimber.getState() == Climber.State.STRAFING && Math.abs(strafe) > Constants.kJoystickJogThreshold) {
                    mLEDs.conformToState(LEDs.State.STRAFING);
                } else {
                    mLEDs.conformToState(LEDs.State.CLIMB_MODE);
                }

//                if (mControlBoard.getStopExtend() || mControlBoard.getStopClimb()) {
//                    climber_action = Climber.WantedAction.STOP;
//                }

                mClimber.setState(climber_action);
            }
            mLEDs.writePeriodicOutputs();
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
            mClimber.setBrakeMode(true);
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

          //  mRobotState.resetVision();

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

            mTurret.setNeutralMode(NeutralMode.Coast);
            mHood.setNeutralMode(NeutralMode.Coast);
            mDrive.setBrakeMode(false);
            mLimelight.writePeriodicOutputs();
            mLEDs.conformToState(LEDs.State.RAINBOW);
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
            mLimelight.writePeriodicOutputs();
           mRoller.setGameData(DriverStation.getInstance().getGameSpecificMessage());

            if (!mLimelight.limelightOK()) {
                mLEDs.conformToState(LEDs.State.EMERGENCY);
            } else if (mTurret.isHoming() || mHood.isHoming()) {
                mLEDs.conformToState(LEDs.State.RAINBOW);
            } else {
                mLEDs.conformToState(LEDs.State.BREATHING_PINK);
            }

            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            mLEDs.writePeriodicOutputs();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
}
