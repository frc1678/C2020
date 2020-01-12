package com.team1678.frc2020.states;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.RobotState;
import com.team254.lib.geometry.Pose2d;

import java.lang.Math;

public class SuperstructureState {
    public double turret; 
    public double hood; 
    public double shooter; 

    public SuperstructureState(double turret, double hood, double shooter) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
    }

    public SuperstructureState(SuperstructureState other) {
        this.turret = other.turret;
        this.hood = other.hood;
        this.shooter = other.shooter;
    }

    // default robot position
    public SuperstructureState() {
        this(0, 0, 0);
    }

    public void setFrom(SuperstructureState source) {
        turret = source.turret;
        hood = source.hood;
        shooter = source.shooter;
    }

    @Override
    public String toString() {
        return "SuperstructureState{" +
                "turret=" + turret +
                ", hood=" + hood +
                ", shooter=" + shooter +
                '}';
    }

    

    public Double[] asVector() {
        return new Double[]{turret, hood, shooter};
    }

    RobotState mRobotState = RobotState.getInstance();

    Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                                                 .transformBy(mRobotState.getPredictedFieldToVehicle(0.7)); //TODO: Find parameter for getPredictedFieldToVehicle()
    Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                                                             .transformBy(mLatestAimingParameters.get().getRobotToGoal());
    
    double mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();
    
    double goal_theta;
    boolean valid_theta;

    double min_theta = Math.toRadians(5.0);
    double max_theta = Math.toRadians(85.0);

    double velocity_iy;
    double first_velocity_ix;
    double velocity_ix;

    double goal_velocity_init;
    double goal_output_velocity;

    double min_velocity_RPM = 2000;
    double max_velocity_RPM = 8000;
    boolean valid_velocity;


    
    public double getVerticalInit() {
        velocity_iy = Math.sqrt(-(2.0 * Constants.g * Constants.kHeightToGoal));

        return velocity_iy;
    }

    public double getFirstHorizInit() {
        first_velocity_ix = Math.sqrt(Math.pow(Constants.kFinalHorizVelocity, 2.0) - (2.0 * Constants.kHorizDragFactor * mCorrectedRangeToTarget));

        return first_velocity_ix;
    }

    public double getHoodAngle() {        
        goal_theta = Math.atan(velocity_iy / first_velocity_ix);

        if (min_theta < goal_theta && goal_theta < max_theta) {
            valid_theta =  true;
        } else { 
            if (goal_theta > max_theta) {
                goal_theta = max_theta;
            } else if (goal_theta < min_theta) {
                goal_theta = min_theta;
            }
        }

        return goal_theta;
    }

    public double getGoalVelocityInit() {
        if (valid_theta) {
            velocity_ix = first_velocity_ix;
        } else {
            velocity_ix = velocity_iy / Math.tan(goal_theta);
        }

        goal_velocity_init = Math.sqrt(Math.pow(velocity_iy, 2.0) + Math.pow(velocity_ix, 2.0));
        goal_output_velocity = ((goal_velocity_init + Constants.kVelocityDrop) / (Constants.kShooterWheelRadiusInches * Constants.kBallSpeedGain)) / Constants.kVelocityConversion;

        if (min_velocity_RPM < goal_output_velocity && goal_output_velocity < max_velocity_RPM) {
            valid_velocity = true;
        } else {
            System.out.println("Cannot shoot from this distance!!");
        }
    }
}
