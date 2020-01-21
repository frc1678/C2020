package com.team1678.frc2020.subsystems;

import java.util.Optional;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

public class Superstructure extends Subsystem {
    // Instances
    private static Superstructure mInstance;

    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    private Rotation2d mFieldRelativeTurretGoal = null;

    enum TurretControlModes {
        FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
    }

    // hood + flywheel
    enum HoodControlModes {
        VISION_AIMED, JOGGING, WAIT_FOR_SPINUP, SHOOTING
    }

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private double mHoodFeedforwardV = 0.0;
    private double mTurretFeedforwardV = 0.0;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;
    private boolean mWantsShoot = false;

    private double mCurrentTurret = 0.0;
    private double mCurrentHood = 0.0;
    private double mCurrentShooter = 0.0;

    private double mTurretSetpoint = 0.0;
    private double mHoodSetpoint = 0.0;
    private double mShooterSetpoint = 0.0;

    private TurretControlModes mTurretMode = TurretControlModes.FIELD_RELATIVE;
    private HoodControlModes mHoodMode = HoodControlModes.VISION_AIMED;

    private double mTurretThrottle = 0.0;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mHoodMode = HoodControlModes.VISION_AIMED;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateCurrentState();
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    followSetpoint();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized boolean isAimed() {
        return mOnTarget;
    }

    private double getShootingSetpointRpm(double range) {
        if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    private double getmHoodSetpointAngle(double range) {
        if (SuperstructureConstants.kUseHoodAutoAimPolynomial) {
            return SuperstructureConstants.kHoodAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    public synchronized boolean isShooting() {
        return mHoodMode == HoodControlModes.SHOOTING;
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized TurretControlModes getTurretControlMode() {
        return mTurretMode;
    }

    // Jog Turret
    public synchronized void jogTurret(double delta) {
        mTurretMode = TurretControlModes.JOGGING;
        double prev_delta = mTurret.getAngle();
        mTurretSetpoint = (prev_delta + delta);
        mTurretFeedforwardV = 0.0;
    }

    // Jog Hood
    public synchronized void JogHood(double delta) {
        mHoodMode = HoodControlModes.JOGGING;
        double prev_delta = mHood.getAngle();
        mHoodSetpoint = (prev_delta + delta);
        mHoodFeedforwardV = 0.0;
    }

    public synchronized void setGoal(double shooter, double hood, double turret) {
        if ((mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget)
                || (mHoodMode == HoodControlModes.VISION_AIMED && mHasTarget)) {
            // Keep current setpoints
        } else {
            mTurretSetpoint = turret;
            mHoodSetpoint = hood;
            mShooterSetpoint = shooter;
        }
    }


    public synchronized void updateCurrentState() {
        mCurrentTurret = mTurret.getAngle();
        mCurrentHood = mHood.getAngle();
        mCurrentShooter = mShooter.getVelocity();
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public void safetyReset() {
        if (mTurretSetpoint < Constants.kTurretConstants.kMinUnitsLimit) {
            mTurretSetpoint += SuperstructureConstants.kTurretDOF;
        }
        if (mTurretSetpoint > Constants.kTurretConstants.kMaxUnitsLimit) {
            mTurretSetpoint -= SuperstructureConstants.kTurretDOF;
        }

        if (mHoodSetpoint < Constants.kHoodConstants.kMinUnitsLimit) {
            // logic for when hood fully in
            mHoodSetpoint = Constants.kHoodConstants.kMinUnitsLimit;
        }
        if (mHoodSetpoint > Constants.kHoodConstants.kMaxUnitsLimit) {
            mHoodSetpoint = Constants.kHoodConstants.kMaxUnitsLimit;
            // logic for when hood fully extended
        }
    }

    public synchronized void maybeUpdateGoalFromVision(double timestamp) {

        if (mTurretMode != TurretControlModes.VISION_AIMED || mHoodMode != HoodControlModes.VISION_AIMED) {
            resetAimingParameters();
            return;
        }

        mLatestAimingParameters = mRobotState.getAimingParameters( -1, Constants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(kLookaheadTime));
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());
            mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            final double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mShooterSetpoint = shooting_setpoint;

            final double aiming_setpoint = getmHoodSetpointAngle(mCorrectedRangeToTarget);
            mHoodSetpoint = aiming_setpoint;

            final Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
            final Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            final double tangential_component = mLatestAimingParameters.get().getRobotToGoalRotation().sin()
                    * velocity.dx / mLatestAimingParameters.get().getRange();
            final double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local
            // frame.
            mTurretFeedforwardV = -(angular_component + tangential_component);

            safetyReset();

            mHasTarget = true;
            final double hood_error = mCurrentHood - mHoodSetpoint;

            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0) && Util.epsilonEquals(hood_error, 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }

        } else {
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    public synchronized void maybeUpdateGoalFromFieldRelativeGoal(double timestamp) {
        if (mTurretMode != TurretControlModes.FIELD_RELATIVE && mTurretMode != TurretControlModes.VISION_AIMED) {
            mFieldRelativeTurretGoal = null;
            return;
        }
        if (mTurretMode == TurretControlModes.VISION_AIMED && !mLatestAimingParameters.isEmpty()) {
            // Vision will control the turret.
            return;
        }
        if (mFieldRelativeTurretGoal == null) {
            return;
        }
        final double kLookaheadTime = 0.7;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(Pose2d.fromRotation(mRobotState.getVehicleToTurret(timestamp))).getRotation().inverse()
                .rotateBy(mFieldRelativeTurretGoal);
        mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
    }

    // god mode on the turret
    public synchronized void setTurretOpenLoop(double throttle) {
        mTurretMode = TurretControlModes.OPEN_LOOP;
        mTurretThrottle = throttle;
    }

    public synchronized void followSetpoint() {
        if (mTurretMode == TurretControlModes.OPEN_LOOP) {
            mTurret.setOpenLoop(mTurretThrottle);
        } else {
            mTurret.setSetpointMotionMagic(mTurretSetpoint);
        }

        mHood.setSetpointPositionPID(mHoodSetpoint, mHoodFeedforwardV);
        mShooter.setVelocity(mShooterSetpoint);

        if (Intake.getInstance().getState() != Intake.State.IDLE) {
            mIndexer.setState(Indexer.WantedAction.INDEX);
        } else if (mHoodMode == HoodControlModes.WAIT_FOR_SPINUP && mOnTarget && mWantsShoot) {
            if (mShooter.spunUp()) {
                mIndexer.setState(Indexer.WantedAction.REVOLVE);
                mHoodMode = HoodControlModes.SHOOTING;
            } else {
                mIndexer.setState(Indexer.WantedAction.INDEX);
            }
        } else if (mShooter.spunUp() && mOnTarget && mWantsShoot) {
            mIndexer.setState(Indexer.WantedAction.REVOLVE);
            mHoodMode = HoodControlModes.SHOOTING;
        } else {
            mIndexer.setState(Indexer.WantedAction.NONE);
        }
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance,
            double min_distance) {
        mTurretMode = TurretControlModes.VISION_AIMED;
        mHoodMode = HoodControlModes.VISION_AIMED;
        mFieldRelativeTurretGoal = field_to_turret_hint;
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    public synchronized void setWantShoot(boolean shoot) {
        mWantsShoot = shoot;
    }

    public synchronized void setWaitForSpinup() {
        mHoodMode = HoodControlModes.WAIT_FOR_SPINUP;
    }

    public synchronized void setWantFieldRelativeTurret(Rotation2d field_to_turret) {
        mTurretMode = TurretControlModes.FIELD_RELATIVE;
        mFieldRelativeTurretGoal = field_to_turret;
    }
}
