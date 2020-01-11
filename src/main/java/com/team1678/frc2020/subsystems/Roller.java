package com.team1678.frc2020.subsystems;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.loops.ILooper;
import com.team1678.frc2020.loops.Loop;

import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.ControlType;

// Control panel manipulator
public class Roller extends Subsystem {
    // Constants
    public static double kRotateVoltage = 3.0; // Assign true value later

    // Motors, solenoids and sensors
    public I2C.Port i2cPort = I2C.Port.kOnboard;
    public ColorSensorV3 mColorSensor;
    private final LazySparkMax mRollerMotor;
    public Solenoid mPopoutSolenoid;

    // Color sensing
    private final ColorMatch mColorMatcher = new ColorMatch();

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

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
        mRollerMotor = SparkMaxFactory.createDefaultSparkMax(Constants.kLeftDriveMasterId); // TODO - Replace '0' with the actual number in the constants file
        mPopoutSolenoid = Constants.makeSolenoidForId(0); // TODO - Replace '0' with actual number

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
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public synchronized void writePeriodicOutputs() {
        mRollerMotor.set(ControlType.kVoltage, mPeriodicIO.roller_demand);
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
                        return;
                    } else {
                        switch(mState) {
                            case IDLE:
                                mPeriodicIO.pop_out_solenoid = false;
                                mPeriodicIO.roller_demand = 0.0;
                                break;
                            case ACHIEVING_ROTATION_CONTROL:
                                // TODO - Add code that will rotate a specific number of rotations
                                mPeriodicIO.pop_out_solenoid = true;
                                mPeriodicIO.roller_demand = kRotateVoltage;
                                break;
                            case ACHIEVING_POSITION_CONTROL:
                                mPeriodicIO.pop_out_solenoid = true;

                                ColorMatchResult match = mColorMatcher.matchClosestColor(mPeriodicIO.detected_color);
                            
                                // TODO - Replace 'kBlueTarget' with actual color goal
                                while (match.color != kBlueTarget) {
                                    mPeriodicIO.roller_demand = kRotateVoltage;
                                }

                                break;
                            default:
                                break;
                        }
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

    public void zeroSensors() {}

    @Override
    public void stop() {
        mRunningManual = true;
        mPeriodicIO.roller_demand = 0.0;
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