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
        { 15.0, 30.0 },
        { 40.5, 46.0 },
        { 59.6, 52.0 },
        { 72.8, 62.0 },
        { 82.3, 65.0 },
        { 94.4, 67.0 },
        { 107.2, 73.0 },
        { 117.6, 74.0 },
        { 129.3, 75.0 },
        { 140.0, 74.5 },
        { 149.2, 75.5 },
        { 154.0, 77.5 },
        { 167.9, 77.5 },
        { 178.0, 78.0 },
        { 184.0, 79.0 },
        { 194.0, 80.0 },
        
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
        { 15.0, 1750 },
        { 40.5, 1750 },
        { 59.6, 1750 },
        { 72.8, 2000 },
        { 82.3, 2100 },
        { 94.4, 2000 },
        { 107.2, 2700 },
        { 117.4, 2700 },
        { 129.3, 2700 },
        { 140.0, 2700 },
        { 149.2, 2700 },
        { 154.0, 2700 },
        { 167.9, 2700 },
        { 178.0, 2800 },
        { 184.0, 2800 },
        { 194.0, 2800 },
        
        // TODO: Fill in with values
    };

    static {
        for (double[] pair : kFlywheelManualRPM) {
            kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelManualRPM, 2);
    }

}
