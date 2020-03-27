/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2020;

import com.team1678.frc2020.loops.Looper;
import com.team1678.frc2020.subsystems.Limelight;
import com.team1678.frc2020.controlboard.ControlBoard;
import com.team1678.frc2020.controlboard.IControlBoard;
import com.team254.lib.wpilib.TimedRobot;
import com.team1678.frc2020.SubsystemManager;
import com.team1678.frc2020.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

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

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Drive mDrive = Drive.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

    //Swerve Drive
    private WheelDrive backRight = new WheelDrive(0, 1, 0);
    private WheelDrive backLeft = new WheelDrive(2, 3, 1);
    private WheelDrive frontRight = new WheelDrive(4, 5, 2);
    private WheelDrive frontLeft = new WheelDrive(6, 7, 3);

    private Swerve swerveDrive = new Swerve(backRight, backLeft, frontRight, frontLeft);

    private Joystick joystick = new Joystick(0);

    public void teleopPeriodic () {
        swerveDrive.drive (joystick.getRawAxis (1), jooystick.getRawAxis (0), joystick.getRawAxis(4));
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(mRobotStateEstimator, mDrive, mLimelight);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mLimelight.setLed(Limelight.LedMode.OFF);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

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
            
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

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
}
