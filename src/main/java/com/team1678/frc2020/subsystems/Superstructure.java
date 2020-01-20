package com.team1678.frc2020.subsystems;

import java.util.Optional;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;
import com.team1678.frc2020.states.SuperstructureConstants;
import com.team1678.frc2020.states.SuperstructureGoal;
import com.team1678.frc2020.states.SuperstructureState;
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

    // Current state = actual measured state of each DOF.
    private SuperstructureState mCurrentState = new SuperstructureState();
    // Current setpoint = output of the planner. May or may not be the final goal.
    private SuperstructureGoal mCurrentSetpoint = null;
    // The goal is the final desired state of the superstructure.
    private SuperstructureGoal mGoal;
    private Rotation2d mFieldRelativeTurretGoal = null;

    private SuperstructureGoal mLastValidGoal;
    private SuperstructureGoal mPreHoodLevelGoal;

    enum TurretControlModes {
        ROBOT_RELATIVE, FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
    }

    // hood + flywheel
    enum HoodControlModes {
        VISION_AIMED, JOGGING, WAIT_FOR_SPINUP, SHOOTING
    }

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private boolean mDisableHood = false;
    private boolean mDisableShooter = false;
    private double mHoodHeightToMaintain = Double.NaN;
    private double mHoodFeedforwardV = 0.0;
    private double mTurretFeedforwardV = 0.0;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    private TurretControlModes mTurretMode = TurretControlModes.ROBOT_RELATIVE;
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
                    if (mGoal == null) {
                        return;
                    }

                    updateCurrentState();
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    updateSetpointFromGoal();

                    if (mCurrentSetpoint != null) {
                        followSetpoint(); // if at desired state, this should stabilize the superstructure at that state
                    }
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

    // Getters
    public synchronized SuperstructureGoal getGoal() {
        return mGoal;
    }

    public synchronized SuperstructureState getCurrentState() {
        return mCurrentState;
    }

    public synchronized SuperstructureGoal getSetpoint() {
        return mCurrentSetpoint;
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

    private double getHoodSetpointAngle(double range) {
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
        if (mGoal == null || mGoal.state == null) {
            return;
        }
        mTurretMode = TurretControlModes.JOGGING;
        mGoal.state.turret = mCurrentState.turret + delta;
        mTurretFeedforwardV = 0.0;
    }

    // Jog Hood
    public synchronized void JogHood(double delta) {
        mHoodMode = HoodControlModes.JOGGING;
        mGoal.state.hood = mCurrentState.hood + delta;
        mHoodFeedforwardV = 0.0;
    }

    public synchronized void setGoal(SuperstructureGoal goal) {
        if (mGoal == null) {
            mGoal = new SuperstructureGoal(goal.state);
        }

        if ((mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget)
                || (mHoodMode == HoodControlModes.VISION_AIMED && mHasTarget)) {
            // Keep current setpoints
        } else {
            mGoal.state.turret = goal.state.turret;
            mGoal.state.shooter = goal.state.shooter;
            mGoal.state.hood = goal.state.hood;
        }
    }

    public synchronized void updateCurrentState() {
        mCurrentState.turret = mTurret.getAngle();
        mCurrentState.hood = mHood.getAngle();
        mCurrentState.shooter = mShooter.getVelocity();
    }

    public synchronized void updateSetpointFromGoal() {
        if (mLastValidGoal == null) {
            // Assume that the current state of the robot must be valid
            mLastValidGoal = new SuperstructureGoal(mCurrentState);
        }

        if (mCurrentSetpoint == null) {
            mCurrentSetpoint = new SuperstructureGoal(mCurrentState);
        }

        if (mTurretMode == TurretControlModes.VISION_AIMED) {
            mCurrentSetpoint = new SuperstructureGoal(mGoal.state);
            mLastValidGoal.state.setFrom(mGoal.state);
        } else {
            if (mCurrentSetpoint.isAtDesiredState(mCurrentState) || !mGoal.equals(mLastValidGoal)) {
                mCurrentSetpoint = mGoal;
            }
        }
        mLastValidGoal.state.setFrom(mGoal.state);
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }


    public void safetyReset() {
        if (mGoal.state.turret < Constants.kTurretConstants.kMinUnitsLimit) {
            mGoal.state.turret += SuperstructureConstants.kTurretDOF;
        }
        if (mGoal.state.turret > Constants.kTurretConstants.kMaxUnitsLimit) {
            mGoal.state.turret -= SuperstructureConstants.kTurretDOF;
        }

        if (mGoal.state.hood < Constants.kHoodConstants.kMinUnitsLimit) {
            // logic for when hood fully in
            mGoal.state.hood = Constants.kHoodConstants.kMinUnitsLimit;
        }
        if (mGoal.state.hood > Constants.kHoodConstants.kMaxUnitsLimit) {
            mGoal.state.hood = Constants.kHoodConstants.kMaxUnitsLimit;
            // logic for when hood fully extended
        }
    }

    public synchronized void maybeUpdateGoalFromVision(double timestamp) {

        if (mTurretMode != TurretControlModes.VISION_AIMED || mHoodMode != HoodControlModes.VISION_AIMED) {
            resetAimingParameters();
            return;
        }

        boolean useHighTarget = mRobotState.useInnerTarget();
        mLatestAimingParameters = mRobotState.getAimingParameters(useHighTarget, -1, Constants.kMaxGoalTrackAge);
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

            double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mGoal.state.shooter = shooting_setpoint;

            double aiming_setpoint = getHoodSetpointAngle(mCorrectedRangeToTarget);
            mGoal.state.hood = aiming_setpoint;

            Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            mGoal.state.turret = mCurrentState.turret + turret_error.getDegrees();
            Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            double tangential_component = mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx
                    / mLatestAimingParameters.get().getRange();
            double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local
            // frame.
            mTurretFeedforwardV = -(angular_component + tangential_component);

            safetyReset();

            Rotation2d hood_error = mRobotState.getVehicleToHood(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            mGoal.state.hood = mCurrentState.hood + hood_error.getDegrees();

            mHasTarget = true;

            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0)
                    && Util.epsilonEquals(hood_error.getDegrees(), 0.0, 3.0)) {
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
            mTurretMode = TurretControlModes.ROBOT_RELATIVE;
            return;
        }
        final double kLookaheadTime = 0.7;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(Pose2d.fromRotation(mRobotState.getVehicleToTurret(timestamp))).getRotation().inverse()
                .rotateBy(mFieldRelativeTurretGoal);
        mGoal.state.turret = mCurrentState.turret + turret_error.getDegrees();
    }

    // god mode on the turret
    public synchronized void setTurretOpenLoop(double throttle) {
        mTurretMode = TurretControlModes.OPEN_LOOP;
        mTurretThrottle = throttle;
    }

    public synchronized void followSetpoint() {

        if (mTurretMode == TurretControlModes.OPEN_LOOP) {
            mTurret.setOpenLoop(mTurretThrottle);
        } else if (mTurretMode == TurretControlModes.VISION_AIMED || mTurretMode == TurretControlModes.JOGGING) {
            mTurret.setSetpointPositionPID(mCurrentSetpoint.state.turret, mTurretFeedforwardV);
        } else {
            mTurret.setSetpointMotionMagic(mCurrentSetpoint.state.turret);
        }

        mHood.setSetpointPositionPID(mCurrentSetpoint.state.hood, mHoodFeedforwardV);
        mShooter.setVelocity(mCurrentSetpoint.state.shooter);
        
        if (Intake.getInstance().getState() != Intake.State.IDLE) {
            mIndexer.setState(Indexer.WantedAction.INDEX);
        } else if (mShooter.spunUp() && mOnTarget) {
            mIndexer.setState(Indexer.WantedAction.REVOLVE);
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

    public synchronized void setWantRobotRelativeTurret() {
        mTurretMode = TurretControlModes.ROBOT_RELATIVE;
    }

    public synchronized void shootCell() {
        mHoodMode = HoodControlModes.WAIT_FOR_SPINUP;
    }

    public synchronized void setWantFieldRelativeTurret(Rotation2d field_to_turret) {
        mTurretMode = TurretControlModes.FIELD_RELATIVE;
        mFieldRelativeTurretGoal = field_to_turret;
    }

}
