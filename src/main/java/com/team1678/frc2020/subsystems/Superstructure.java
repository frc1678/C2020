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
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    // Instances
    private static Superstructure mInstance;

    private final Turret mTurret = Turret.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Trigger mTrigger = Trigger.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();
    private boolean mAutoIndex = false;

    private Rotation2d mFieldRelativeTurretGoal = null;

    enum TurretControlModes {
        FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
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
    private boolean mWantsSpinUp = false;
    private boolean mWantsTuck = false;
    private boolean mSettled = false;
    private boolean mUseInnerTarget = false;

    private double mCurrentTurret = 0.0;
    private double mCurrentHood = 0.0;

    private double mTurretSetpoint = 0.0;
    private double mHoodSetpoint = 82.5;
    private double mShooterSetpoint = 2700.0;
    private boolean mGotSpunUp = false;

    private TurretControlModes mTurretMode = TurretControlModes.FIELD_RELATIVE;

    private double mTurretThrottle = 0.0;
    private TimeDelayedBoolean trigger_popout = new TimeDelayedBoolean();
    private boolean estim_popout = false;

    public synchronized boolean spunUp() {
        return mGotSpunUp;
    }
    
    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
    }

    public boolean getWantShoot() {
        return mWantsShoot;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mTurretMode = TurretControlModes.FIELD_RELATIVE;
                    if (SuperstructureConstants.kUseSmartdashboard) {
                        SmartDashboard.putNumber("Shooting RPM", mShooterSetpoint);
                        SmartDashboard.putNumber("Hood Angle", mHoodSetpoint);
                    }
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

    public synchronized boolean getWantsShoot() {
        return mWantsShoot;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Turret Control State", mTurretMode.toString());
        SmartDashboard.putNumber("Turret Goal", mTurretSetpoint);
        SmartDashboard.putNumber("Hood Goal", mHoodSetpoint);
        SmartDashboard.putBoolean("Stopped at Deadspot", mSettled);
        SmartDashboard.putBoolean("Spun Up", mGotSpunUp);
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
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (SuperstructureConstants.kUseFlywheelAutoAimPolynomial) {
            return SuperstructureConstants.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    private double getHoodSetpointAngle(double range) {
        if (SuperstructureConstants.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (SuperstructureConstants.kUseHoodAutoAimPolynomial) {
            return SuperstructureConstants.kHoodAutoAimPolynomial.predict(range);
        } else {
            return SuperstructureConstants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
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
        double prev_delta = mHood.getAngle();
        mHoodSetpoint = (prev_delta + delta);
        mHoodFeedforwardV = 0.0;
    }

    public synchronized void setGoal(double shooter, double hood, double turret) {
        if ((mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget)) {
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

        if (mTurretMode != TurretControlModes.VISION_AIMED) {
            resetAimingParameters();
            return;
        }

        if (mWantsShoot) {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, mTrackId, Constants.kMaxGoalTrackAge);
        } else {
            mLatestAimingParameters = mRobotState.getAimingParameters(mUseInnerTarget, -1, Constants.kMaxGoalTrackAge);
        }

        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime));
            Pose2d predicted_turret_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getTurretToGoal());
            mCorrectedRangeToTarget = predicted_turret_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            final double shooting_setpoint = getShootingSetpointRpm(mCorrectedRangeToTarget);
            mShooterSetpoint = shooting_setpoint;

            final double aiming_setpoint = getHoodSetpointAngle(mCorrectedRangeToTarget);
            mHoodSetpoint = aiming_setpoint;

            final Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getTurretToGoalRotation());
            
            mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
            final Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            final double tangential_component = mLatestAimingParameters.get().getTurretToGoalRotation().sin()
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
        final double kLookaheadTime = 6.0;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(mRobotState.getVehicleToTurret(timestamp)).getRotation().inverse()
                .rotateBy(mFieldRelativeTurretGoal);
        mTurretSetpoint = mCurrentTurret + turret_error.getDegrees();
        safetyReset();
    }

    // god mode on the turret
    public synchronized void setTurretOpenLoop(double throttle) {
        mTurretMode = TurretControlModes.OPEN_LOOP;
        mTurretThrottle = throttle;
    }

    public synchronized void followSetpoint() {
  
        if (SuperstructureConstants.kUseSmartdashboard) {
            mShooterSetpoint = getShootingSetpointRpm(0);
            mHoodSetpoint = getHoodSetpointAngle(0);
        }

        if (mWantsTuck) {
            mHood.setSetpointMotionMagic(0.0);
        } else {
            mHood.setSetpointMotionMagic(mHoodSetpoint);
        }

        Indexer.WantedAction indexerAction = Indexer.WantedAction.PREP;
        double real_trigger = 0.0;
        double real_shooter = 0.0;
        boolean real_popout = false;

        if (Intake.getInstance().getState() == Intake.State.INTAKING) {
            if (mAutoIndex) {
                indexerAction = Indexer.WantedAction.INDEX;
            } else {
                indexerAction = Indexer.WantedAction.PASSIVE_INDEX;
            }
        }

        if (mWantsSpinUp) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.PASSIVE_INDEX;
            real_trigger = -600.0;
        } else if (mWantsShoot) {
            real_shooter = mShooterSetpoint;
            indexerAction = Indexer.WantedAction.PREP;
            real_trigger = Constants.kTriggerRPM;
            
            if (mSettled) {
                real_popout = true;
            }

            if (mIndexer.isAtDeadSpot() && Math.abs((mTurret.getVelocity() / 360. / 60.) - mIndexer.getIndexerVelocity()) < 1) {
                mSettled = true;
            }

            /*if (mSettled) {
                real_popout = true;
                real_trigger = Constants.kTriggerRPM;
                if (mShooter.spunUp() && mTrigger.spunUp()) {
                    mGotSpunUp = true;
                }
            }*/

            if (mShooter.spunUp() && mTrigger.spunUp()) {
                mGotSpunUp = true;
            }

            if (mGotSpunUp && estim_popout) {
                //real_popout = true;
                //real_trigger = Constants.kTriggerRPM;
                indexerAction = Indexer.WantedAction.ZOOM;
            }
        }

        mIndexer.setState(indexerAction);
        mTrigger.setPopoutSolenoid(real_popout);
        mTrigger.setVelocity(real_trigger);
        if (real_shooter < Util.kEpsilon) {
            mShooter.setOpenLoop(0);
        } else {
            mShooter.setVelocity(real_shooter);
        }

        if (mTurretMode == TurretControlModes.OPEN_LOOP) {
            mTurret.setOpenLoop(mTurretThrottle);
        //} else  if (mTurretMode == TurretControlModes.VISION_AIMED) {
         //   mTurret.setSetpointPositionPID(mTurretSetpoint, mTurretFeedforwardV);
        } else {
            mTurret.setSetpointMotionMagic(mTurretSetpoint);
        }
        //mTurret.setOpenLoop(0);
        //mHood.setOpenLoop(0);
        estim_popout = trigger_popout.update(real_popout, 0.2);
    }

    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }
    
    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance,
            double min_distance) {
        mTurretMode = TurretControlModes.VISION_AIMED;
        mFieldRelativeTurretGoal = field_to_turret_hint;
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    public synchronized void setWantShoot() {
        mWantsSpinUp = false;
        mWantsShoot = !mWantsShoot;
        mSettled = false;
        mGotSpunUp = false;
    }

    public synchronized void setWantSpinUp() {
        mWantsSpinUp = !mWantsSpinUp;
        mWantsShoot = false;
        mSettled = false;
        mGotSpunUp = false;
    }

    public synchronized void setWantShoot(boolean shoot) {
        mWantsSpinUp = false;
        mWantsShoot = shoot;
        mSettled = false;
        mGotSpunUp = false;
    }

    public synchronized void setWantSpinUp(boolean spin_up) {
        mWantsSpinUp = spin_up;
        mWantsShoot = false;
    }

    public synchronized void setWantTuck() {
        mWantsTuck = !mWantsTuck;
    }

    public synchronized void setWantInnerTarget(boolean inner) {
        mUseInnerTarget = inner;
    }

    public synchronized void setAutoIndex(boolean auto_index) {
        mAutoIndex = auto_index;
    }

    public synchronized void setWantFieldRelativeTurret(Rotation2d field_to_turret) {
        mTurretMode = TurretControlModes.FIELD_RELATIVE;
        mFieldRelativeTurretGoal = field_to_turret;
    }
}
