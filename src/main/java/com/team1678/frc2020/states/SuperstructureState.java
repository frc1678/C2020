package com.team1678.frc2020.states;

import com.team254.lib.util.Util;

public class SuperstructureState {
    public double turret; // degrees
    public double shooter; // velocity
    public double hood; // degrees

    public SuperstructureState(double turret, double shooter, double hood) {
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
    }

    public SuperstructureState(SuperstructureState other) {
        this.turret = other.turret;
        this.shooter = other.shooter;
        this.hood = other.hood;
    }

    // default robot position
    public SuperstructureState() {
        this(0, 0, 0);
    }

    public void setFrom(SuperstructureState source) {
        turret = source.turret;
        shooter = source.shooter;
        hood = source.hood;
    }

    public boolean isTurretSafeForWristBelowBumper() {
        return Util.epsilonEquals(turret, 0, SuperstructureConstants.kTurretPaddingDegrees)
                || Util.epsilonEquals(turret, 180, SuperstructureConstants.kTurretPaddingDegrees);
    }

    @Override
    public String toString() {
        return "SuperstructureState{" +
                "turret=" + turret +
                ", shooter=" + shooter +
                ", hood=" + hood +
                '}';
    }

    public Double[] asVector() {
        return new Double[]{turret, shooter, hood};
    }
}
