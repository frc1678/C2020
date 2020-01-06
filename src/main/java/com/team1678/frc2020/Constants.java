package com.team1678.frc2020;

import com.team1678.frc2020.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2020.subsystems.ServoMotorSubsystem.TalonSRXConstants;
import com.team1678.frc2020.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.02;
    public static final boolean kDebuggingOutput = false;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    /* ROBOT PHYSICAL CONSTANTS */
    // Wheels
    public static final double kDriveWheelTrackWidthInches = 27.75;
    public static final double kDriveWheelDiameterInches = 4.0;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 12.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 0.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.44;  // V
    public static final double kDriveKv = 0.129;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    // PID gains for drive velocity loop 
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveVelocityKp = 0.1;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 1.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // drive
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveId = 2;

    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveSlaveId = 4;

    // pigeon
    public static final int kPigeonIMUId = 15;

    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;

    // solenoids
    public static final int kPCMId = 1;
    public static final int kShifterSolenoidId = 7;
    public static final int kBallIntakeJawId = 6;
    public static final int kKickstandForwardId = 1; // deploys
    public static final int kKickstandReverseId = 0; // retracts
    public static final int kRatchetForwardId = 3; // deploys
    public static final int kRatchetReverseId = 2; // retracts

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    // Top limelight
    public static final LimelightConstants kTopLimelightConstants = new LimelightConstants();
    static {
        kTopLimelightConstants.kName = "Top Limelight";
        kTopLimelightConstants.kTableName = "limelight-top";
        kTopLimelightConstants.kHeight = 44.047;  // inches
        kTopLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-7.685, 0.0), Rotation2d.fromDegrees(0.0));
        kTopLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
    }

    // Bottom limelight
    public static final LimelightConstants kBottomLimelightConstants = new LimelightConstants();
    static {
        kBottomLimelightConstants.kName = "Bottom Limelight";
        kBottomLimelightConstants.kTableName = "limelight-bottom";
        kBottomLimelightConstants.kHeight = 7.221;  // inches
        kBottomLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-1.293, 2.556), Rotation2d.fromDegrees(2.0));
        kBottomLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(47.5);
    }

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

    // Drive control
    public static final double kStingerForwardPower = 0.8;
    public static final double kClimbingElevatorHeightForLowShift = 10.0; // in

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}