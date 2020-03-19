package com.team1323.lib.geometry;

import com.team254.lib.geometry.State;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
