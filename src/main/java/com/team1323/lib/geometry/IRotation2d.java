package com.team1323.lib.geometry;

import com.team254.lib.geometry.State;

public interface IRotation2d<S> extends State<S> {
    public UnwrappableRotation2d getRotation();
}
