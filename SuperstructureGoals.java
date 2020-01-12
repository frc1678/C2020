package com.team1678.frc2020;

public class SuperstructureGoal {
    public final SuperstructureState state;

    public SuperstructureGoal(double turret, double indexer, double hood, double shooter){
        this(new SuperstructureState(turret, indexer, hood, shooter));
    }

    public SupersturctureGoal(SuperstructureState state) {
        this.state = new SuperstructureState(state);
    }

    public boolean equals(SuperstructureGoal other) {
        return this.state.turret == other.state.turret &&
        this.state.indexer == other.state.indexer &&
        this.state.hood == other.state.hood &&
        this.state.shooter == other.state.shooter;
    }

    public boolean isAtDesiredState(SuperstructureStates currentState) {
        double[] distances = {
            currentState.turret - state.turret,
            currentState.indexer - state.indexer,
            currentState.hood - state.hood,
            currentState.shooter - state.shooter
        };

        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperstructureConstants.kPadding[i]) {
                return false;
            }
        }
        return true;
    }

}