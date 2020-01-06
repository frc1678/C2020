package com.team1678.frc2020.states;

public class ClimberConstants { // TODO(Hanson)
    public static double kElevatorMinHeight = 0.0;
    public static double kElevatorMaxHeight = 69.0;

    // Safety
    public static final double kElevatorApproachingThreshold = 1.0;

    // Godmode
    // This is in inches / ~20ms
    public final static double kElevatorGodmodeThrottle = 60.0 / 50.0;

    // elevator constants
    public static double kGroundHeight = 0.;
    public static double kReachHeight = 63.;
    public static double kHookHeight = 60.;
    public static double kAdjustHeight = 6.;
    public static double kClimbHeight = 2.;

    public static double kElevatorRezeroCurrentThreshold = 3;  // tune
}   
