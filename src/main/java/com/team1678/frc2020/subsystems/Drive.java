package com.team1678.frc2020.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team1678.frc2020.Constants;
import com.team1678.frc2020.Kinematics;
import com.team1678.frc2020.RobotState;
import com.team1678.frc2020.logger.*;
import com.team1678.frc2020.logger.LogStorage;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.planners.DriveMotionPlanner;
import com.team1678.lib.control.PIDController;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonFXChecker;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;


public class Drive extends Subsystem {

    private static final int kVelocityControlSlot = 0;
    private static final double DRIVE_ENCODER_PPR = 2048. * 10.;
    private static Drive mInstance = new Drive();
    // Hardware
    private final TalonFX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory = false;
    private double mWheelNonzeroTimestamp = 0.0;
    private final PIDController steeringStabilizer = new PIDController(3.0, 0.0, 0.1, 0.05);
    
    private boolean mHasResetSteering = false;
    private boolean mStartedResetTimer = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
                mStartedResetTimer = false;
                mHasResetSteering = false;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    case CLOSED_LOOP:
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            stopLogging();
        }
    };



    public LogStorage mStorage = null;

    public void logSetup() {
        mStorage = new LogStorage();
        ArrayList<String> columnNames = new ArrayList<String>();

        //  OUTPUTS
        columnNames.add("timestamp");
        columnNames.add("left_position_ticks");
        columnNames.add("right_position_ticks");
        columnNames.add("left_distance");
        columnNames.add("right_distance");
        columnNames.add("left_current");
        columnNames.add("right_current");
        columnNames.add("left_velocity_ticks_per_100ms");
        columnNames.add("right_velocity_ticks_per_100ms");
        columnNames.add("gyro_heading");

        //  OUTPUTS
        columnNames.add("left_demand");
        columnNames.add("right_demand");
        columnNames.add("left_accel");
        columnNames.add("right_accel");
        columnNames.add("left_feedforward");
        columnNames.add("right_feedforward");
        columnNames.add("path_setpoint");

        mStorage.setHeaders(columnNames);
    }

    
    public void logWrite() {
        ArrayList<Double> items = new ArrayList<Double>();

        //  INPUTS
        items.add(Timer.getFPGATimestamp());
        items.add(Double.valueOf(mPeriodicIO.left_position_ticks));
        items.add(Double.valueOf(mPeriodicIO.right_position_ticks));
        items.add(mPeriodicIO.left_distance);
        items.add(mPeriodicIO.right_distance);
        items.add(mPeriodicIO.left_current);
        items.add(mPeriodicIO.right_current);
        items.add(Double.valueOf(mPeriodicIO.left_velocity_ticks_per_100ms));
        items.add(Double.valueOf(mPeriodicIO.right_velocity_ticks_per_100ms));
        items.add(mPeriodicIO.gyro_heading.getDegrees());

        //  OUTPUTS
        items.add(mPeriodicIO.left_demand);
        items.add(mPeriodicIO.right_demand);
        items.add(mPeriodicIO.left_accel);
        items.add(mPeriodicIO.right_accel);
        items.add(mPeriodicIO.left_feedforward);
        items.add(mPeriodicIO.right_feedforward);
        //items.add(Double.valueOf(mPeriodicIO.path_setpoint.toString()));

        try {
            mStorage.addData(items);
        } catch(Exception e) {}
    }

    public void registerLogger(LoggingSystem LS) {
        logSetup();
        LS.register(mStorage, "Drive.csv");
    }
    private void configureMaster(TalonFX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .IntegratedSensor, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(!left);
        talon.setSensorPhase(false);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();
        startLogging();

        // Start all Talons in open loop mode.
        mLeftMaster = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        configureMaster(mLeftMaster, true);

        mLeftSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveId,
                Constants.kLeftDriveMasterId);
        mLeftSlave.setInverted(false);

        mRightMaster = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        configureMaster(mRightMaster, false);

        mRightSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveId,
                Constants.kRightDriveMasterId);
        mRightSlave.setInverted(true);

        reloadGains();

        mPigeon = new PigeonIMU(Constants.kPigeonIMUId);
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, 10);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance() {
        return mInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s /(Math.PI * 2.0) * (DRIVE_ENCODER_PPR) / 10.0;
    }

    private static double inchesPerSecondToTicksPer100ms(double in_s) {
        return radiansPerSecondToTicksPer100ms(in_s / Constants.kDriveWheelRadiusInches);
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);

            System.out.println("Switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }    
    
    public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.1;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void setAssistedDrive(double timestamp, double throttle, double wheel, boolean quickTurn) {
        
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 7.0;
        final double kWheelNonlinearity = 0.01;
        final double kMaxCurvature = 1.0 / 27.0;
        final double kMaxLinearVel = 144.0;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        throttle *= kMaxLinearVel;
        if (!quickTurn) {
            Util.limit(wheel, throttle * kMaxCurvature);
        }
        final double heading = RobotState.getInstance().getFieldToVehicle(timestamp).getRotation().getRadians();
        double correction = steeringStabilizer.update(timestamp, heading);
        if (!mStartedResetTimer && Math.abs(wheel) < Util.kEpsilon) {
            mWheelNonzeroTimestamp = timestamp;
            mStartedResetTimer = true;
        }

        if (Math.abs(wheel) > Util.kEpsilon) {
            mStartedResetTimer = false;
        }

        if ((timestamp - mWheelNonzeroTimestamp) < 0.5 || Math.abs(wheel) > Util.kEpsilon) {
            correction = 0;
            mHasResetSteering = false;
        } else if (!mHasResetSteering) {
            correction = 0;
            steeringStabilizer.setGoal(heading);
            mHasResetSteering = true;
        }

        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel + correction));
        setClosedLoop(new DriveSignal(Util.limit(signal.getLeft(), kMaxLinearVel) / 2.0, Util.limit(signal.getRight(), kMaxLinearVel) / 2.0));
    }

    public synchronized void setClosedLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.CLOSED_LOOP) {
            // We entered a velocity control state.
            setBrakeMode(false);
            mLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.CLOSED_LOOP;
        }
        mPeriodicIO.left_demand = radiansPerSecondToTicksPer100ms(signal.getLeft());
        mPeriodicIO.right_demand = radiansPerSecondToTicksPer100ms(signal.getRight());
        mPeriodicIO.left_feedforward = (signal.getLeft() * Constants.kDriveKv) / 12.0;
        mPeriodicIO.right_feedforward = (signal.getRight() * Constants.kDriveKv) / 12.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand =  signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if(mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if(mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(-mPigeon.getFusedHeading()).inverse());
     //  System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        logWrite();

        if(getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (Constants.kDebuggingOutput) {
            SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
            SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);

            SmartDashboard.putNumber("Left Current", mPeriodicIO.left_current);
            SmartDashboard.putNumber("Right Current", mPeriodicIO.right_current);

            SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
            SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
            SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
        }
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if(!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(-mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / (DRIVE_ENCODER_PPR)) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / (DRIVE_ENCODER_PPR)) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }

        mPeriodicIO.left_current = mLeftMaster.getOutputCurrent();
        mPeriodicIO.right_current = mRightMaster.getOutputCurrent();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
        } else if (mDriveControlState == DriveControlState.CLOSED_LOOP) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward);
        }
    }

    @Override
    public boolean checkSystem() {
        boolean leftSide = TalonFXChecker.checkMotors(this,
        new ArrayList<MotorChecker.MotorConfig<TalonFX>>() {
                    private static final long serialVersionUID = 4715363468641125563L;
                    {
                        add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster));
                        add(new MotorChecker.MotorConfig<>("left_slave", mLeftSlave));
                    }
                }, new TalonFXChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
                    }
                });
        boolean rightSide = TalonFXChecker.checkMotors(this,
        new ArrayList<MotorChecker.MotorConfig<TalonFX>>() {
                    private static final long serialVersionUID = 8979637825679409635L;
                    {
                        add(new MotorChecker.MotorConfig<>("right_master", mRightMaster));
                        add(new MotorChecker.MotorConfig<>("right_slave", mRightSlave));
                    }
                }, new TalonFXChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
                    }
                });
        return leftSide && rightSide;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    // The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        CLOSED_LOOP, // teleop velocity control
    }

    public static class PeriodicIO {
        public double timestamp;

        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public double left_current;
        public double right_current;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
  
}
