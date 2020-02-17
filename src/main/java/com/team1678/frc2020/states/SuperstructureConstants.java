package com.team1678.frc2020.states;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.InterpolatingDouble;



public class SuperstructureConstants {
    public static final double kTurretPaddingDegrees = 3;
    public static final double kHoodPaddingDegrees = 2;
    public static final double kShooterPaddingVelocity = 100;


    public static final double[] kPadding = {
            kTurretPaddingDegrees, kShooterPaddingVelocity, kHoodPaddingDegrees};

    //turret
    public static final double kTurretDOF = 360;


    //hood
    public static double kDefaultHoodAngle = Math.toRadians(0);
    public static boolean kUseHoodAutoAimPolynomial = false;

    public static boolean kUseSmartdashboard = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kHoodAutoAimPolynomial;

    public static double[][] kHoodManualAngle = {
        { 8.9, 30.0 },
        { 39.0, 42.0 },
        { 56.2, 54.0 },
        { 69.7, 60.0 },
        { 76.1, 62.0 },
        { 80.8, 66.0 },
        { 95.2, 72.0 },
        { 129.3, 75.0 },
        { 132.3, 76.0 },
        { 140.2, 75.5 },
        { 154.0, 78.0 },
        { 167.9, 78.0 },
        { 184.0, 79.0 },
        { 194.0, 80.0 },
        { 217.0, 80.0 },
        
    };

    static {
        //iterate through the array and place each point into the interpolating tree
        for (double[] pair : kHoodManualAngle) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        
        kHoodAutoAimPolynomial = new PolynomialRegression(kHoodManualAngle, 1);
    }
    
    //shooter
    public static double kDefaultShootingRPM = 2950.0;
    public static boolean kUseFlywheelAutoAimPolynomial = false;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static PolynomialRegression kFlywheelAutoAimPolynomial;

    public static double[][] kFlywheelManualRPM = {
        { 8.9, 2200 },
        { 49.0, 2300 },
        { 56.2, 2300 },                
        { 69.7, 2400.0 },
        { 76.1, 2400 },
        { 95.2, 2700 },
        { 129.3, 2700 },
        { 132.3, 2700 },
        { 140.2, 2700 },
        { 154.0, 2700 },
        { 167.9, 2700 },
        { 184.0, 2700 },
        { 194.0, 2700 },
        { 217.0, 2700 },
        { 80.8, 2400 },
        
        // TODO: Fill in with values
    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
