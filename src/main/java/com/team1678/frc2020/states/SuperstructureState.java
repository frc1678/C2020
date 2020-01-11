package com.team1678.frc2020;

public class SuperstructureState {
    public double turret; 
    public double hood; 
    public double shooter; 
    public double indexer; 

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
}