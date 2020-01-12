package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.Spark;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ControlType;

// Control panel manipulator
public class Roller extends Subsystem {
    // Constants
    public static double kRotateVoltage = 3.0; // Assign true value later

    // Motors, solenoids and sensors
    public I2C.Port i2cPort = I2C.Port.kOnboard;
    public ColorSensorV3 mColorSensor;
    private final Spark mRollerMotor;
    public Solenoid mPopoutSolenoid;

    // Color sensing
    private final ColorMatch mColorMatcher = new ColorMatch();
    ColorMatchResult match;

    private final Color kBlueTarget = ColorMatch.makeColor(0.250, 0.691, 0.633);
    private final Color kGreenTarget = ColorMatch.makeColor(0.225, 0.604, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.728, 0.410, 0.132);
    private final Color kYellowTarget = ColorMatch.makeColor(0.871, 0.1194, 0.234);
    private Color mColorPositionTarget;

    int colorCounter;

    Color previousColor;
    Color initialColor;

    String gameData;

    // State management
    public enum WantedAction {
        NONE, ACHIEVE_ROTATION_CONTROL, ACHIEVE_POSITION_CONTROL
    }

    private enum State {
        IDLE, ACHIEVING_ROTATION_CONTROL, ACHIEVING_POSITION_CONTROL
    }

    private State mState = State.IDLE;

    // General management
    private static Roller mInstance;
    private boolean mRunningManual = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Roller() {
        mRollerMotor = new Spark(Constants.kRollerId);
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kRollerSolenoid);

        mColorSensor = new ColorSensorV3(i2cPort);

        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);    
    }

    public synchronized static Roller getInstance() {
        if (mInstance == null) {
            mInstance = new Roller();
        }

        return mInstance;
    }

    public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.detected_color = mColorSensor.getColor();
        gameData = DriverStation.getInstance().getGameSpecificMessage();

        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B' :
                    mColorPositionTarget = kBlueTarget;
                    break;
                case 'G' :
                    mColorPositionTarget = kGreenTarget;
                    break;
                case 'R' :
                    mColorPositionTarget = kRedTarget;
                    break;
                case 'Y' :
                    mColorPositionTarget = kYellowTarget;
                    break;
                default :
                    System.out.println("Invalid color from FMS!");
                    break;
            }
        } else {
            //Code for no data received yet
        }
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public synchronized void writePeriodicOutputs() {
        mRollerMotor.set(mPeriodicIO.roller_demand / 12.0);
        mPopoutSolenoid.set(mPeriodicIO.pop_out_solenoid);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRunningManual = false;
                mState = State.IDLE;
            } 

            @Override 
            public void onLoop(double timestamp) {
                synchronized (Roller.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.IDLE;
            }

        });
    }

    // TODO: Only perform actions if 'modifyOutputs' is true
    public void runStateMachine(boolean modifyOutputs) {
        switch(mState) {
            case IDLE:
                mPeriodicIO.pop_out_solenoid = false;
                mPeriodicIO.roller_demand = 0.0;
                break;
            case ACHIEVING_ROTATION_CONTROL:
                mPeriodicIO.pop_out_solenoid = true;
                match = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                // Run roller until it has gone over the inital color four times
                if (colorCounter < 7) {
                    mPeriodicIO.roller_demand = kRotateVoltage;

                    if (match.color != previousColor) {
                        if (match.color == initialColor && match.confidence > .93) {
                            colorCounter++;
                        }
                    }
                }

                previousColor = match.color;

                // If it has run over the initial color four times, turn off roller
                if (colorCounter >= 7) {
                    setState(WantedAction.NONE);
                    colorCounter = 0;
                }

                break;
            case ACHIEVING_POSITION_CONTROL:
                if (mColorPositionTarget != null) {
                    mPeriodicIO.pop_out_solenoid = true;
                    match = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                    if (match.color == mColorPositionTarget) {
                        setState(WantedAction.NONE);
                    } else {
                        if (match.color != mColorPositionTarget) {
                            mPeriodicIO.roller_demand = kRotateVoltage;
                        }
                    }
                } else {
                    // There is no color goal, do nothing
                }

                break;
            default:
                break;
        }
    }

    public void zeroSensors() {}

    @Override
    public void stop() {
        mRunningManual = true;
        mPeriodicIO.roller_demand = 0.0;
        mPeriodicIO.pop_out_solenoid = false;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Red", mPeriodicIO.detected_color.red);
        SmartDashboard.putNumber("Green", mPeriodicIO.detected_color.green);
        SmartDashboard.putNumber("Blue", mPeriodicIO.detected_color.blue);
    }

    public void setState(WantedAction action) {
        mRunningManual = false;

        switch(action) {
            case NONE:
                mState = State.IDLE;
                break;
            case ACHIEVE_ROTATION_CONTROL:
                match = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);
                initialColor = match.color;
                previousColor = match.color;
                mState = State.ACHIEVING_ROTATION_CONTROL;
                break;
            case ACHIEVE_POSITION_CONTROL:
                mState = State.ACHIEVING_POSITION_CONTROL;
                break;
            default:
                break;
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public Color detected_color;

        // OUTPUTS
        public double roller_demand;
        public boolean pop_out_solenoid;
    }
}