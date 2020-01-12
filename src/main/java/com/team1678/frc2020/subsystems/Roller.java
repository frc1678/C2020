package com.team1678.frc2020.subsystems;

import java.util.stream.Stream;

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
    ColorMatchResult mMatch;

    private final Color kBlueTarget = ColorMatch.makeColor(0.134, 0.432, 0.434);
    private final Color kGreenTarget = ColorMatch.makeColor(0.178, 0.571, 0.251);
    private final Color kRedTarget = ColorMatch.makeColor(0.485, 0.364, 0.150);
    private final Color kYellowTarget = ColorMatch.makeColor(0.314, 0.553, 0.120);

    private int mColorCounter;

    private Color mInitialColor;
    private Color mPreviousColor;
    private Color mPreviousPreviousColor;

    private Color mColorPositionTarget;
    private Color mSlowDownTarget;

    // Game data
    private String gameData;

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
        
        mColorCounter = 0;
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
            // Accounts for the FMS detecting the color two wedges down
            switch (gameData.charAt(0)) {
                case 'B' :
                    mColorPositionTarget = kRedTarget;
                    mSlowDownTarget = kGreenTarget;
                    //System.out.println("Color is blue");
                    break;
                case 'G' :
                    mColorPositionTarget = kYellowTarget;
                    mSlowDownTarget = kRedTarget;
                    break;
                case 'R' :
                    mColorPositionTarget = kBlueTarget;
                    mSlowDownTarget = kYellowTarget;
                    break;
                case 'Y' :
                    mColorPositionTarget = kGreenTarget;
                    mSlowDownTarget = kBlueTarget;
                    break;
                default :
                    System.out.println("Invalid color from FMS!");
                    break;
            }
        } else {
            // No data has been recieved
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
                mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                if (mColorCounter < 7) {
                    mPeriodicIO.roller_demand = kRotateVoltage;
            
                    if (mMatch.color != mPreviousColor) {
                      if (mMatch.color == mInitialColor) {
                        // Necessary to avoid color confusion between red/green and blue/yellow
                        if (mInitialColor == kYellowTarget && mMatch.color == kYellowTarget && mPreviousColor == kGreenTarget && mPreviousPreviousColor == kBlueTarget) {
                          // Do nothing
                        } else if (mInitialColor == kGreenTarget && mMatch.color == kGreenTarget && mPreviousColor == kYellowTarget && mPreviousPreviousColor == kRedTarget) {
                          // Do nothing
                        } else {
                          mColorCounter++;
                        }
                      }
                    }
                  }
            
                  if (mColorCounter >= 7) {
                    mColorCounter = 0;
                    setState(WantedAction.NONE);
                  }

                  if (mPreviousColor != mMatch.color) {
                    mPreviousPreviousColor = mPreviousColor;
                    mPreviousColor = mMatch.color;
                  }

                break;
            case ACHIEVING_POSITION_CONTROL:
                mMatch = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);

                if (gameData.length() > 0) {
                    if (mMatch.color != mColorPositionTarget) {
                        mPeriodicIO.roller_demand = 3.0;
            
                        if (mMatch.color == mSlowDownTarget) {
                            mPeriodicIO.roller_demand = 1.0;
                        }
                    } else {
                        mColorCounter = 0;
                        setState(WantedAction.NONE);
                    }
                }

                break;
            default:
                System.out.println("Invalid roller goal!");
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
                mInitialColor = mMatch.color;
                mPreviousColor = mMatch.color;
                mPreviousColor = mMatch.color;
                mState = State.ACHIEVING_ROTATION_CONTROL;
                break;
            case ACHIEVE_POSITION_CONTROL:
                mState = State.ACHIEVING_POSITION_CONTROL;
                break;
            default:
                System.out.println("Invalid roller action!");
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